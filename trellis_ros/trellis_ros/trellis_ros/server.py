import os
import time
import threading
import traceback

# os.environ['ATTN_BACKEND'] = 'xformers'   # Can be 'flash-attn' or 'xformers', default is 'flash-attn'
os.environ['SPCONV_ALGO'] = 'native'        # Can be 'native' or 'auto', default is 'auto'.
                                            # 'auto' is faster but will do benchmarking at the beginning.
                                            # Recommended to set to 'native' if run only once.

import imageio
from PIL import Image
from trellis.pipelines import TrellisImageTo3DPipeline
from trellis.utils import render_utils, postprocessing_utils

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rcl_interfaces.msg import Parameter, ParameterType, SetParametersResult
from rcl_interfaces.srv import SetParameters

from trellis_msgs.srv import Synthesize3DAsset


def get_current_milliseconds():
    """Get current time in milliseconds."""
    return round(time.time()*1000)

class TrellisServer(Node):

    def __init__(self):
        """Initialize the base class."""
        super().__init__('trellis_server')

        self._status_callback_group = ReentrantCallbackGroup()
        self._process_callback_group = MutuallyExclusiveCallbackGroup()

        self._buffer_dir = None
        self._pipeline_mutex = threading.Lock()
        self._pipeline = None
        self._tp_conversion_service = None
        self._conversion_model = None

        self.declare_parameters(
            namespace='',
            parameters=[
                ("buffer_dir", "/tmp"),
                ("conversion_service", "~/generate"),
                ("conversion_model", "JeffreyXiang/TRELLIS-image-large"),
                ("qsize", 1)
            ]
        )

    def __params_cb__(self, data):
        """Update parameters callback."""
        try:
            params_to_set = [param for param in data if self.has_parameter(param.name)]
            for param in params_to_set:
                if param.name == "conversion_model":
                    self._conversion_model = param.value
                    # reload model
                    with self._pipeline_mutex:
                        self._pipeline = TrellisImageTo3DPipeline.from_pretrained(self._conversion_model)

            self.get_logger().info(f"parameters changed: {[f'{param.name} = {param.value}' for param in params_to_set]}")
            return SetParametersResult(successful=True)

        except ValueError as err:
            self.get_logger().error(f"[TrellisServer] Error: {err}")
        except Exception as err: # pylint: disable=broad-except
            self.get_logger().error(f"[TrellisServer] Error: {err} | {traceback.format_exc()}")
        return SetParametersResult(successful=False)

    def __init_parameters__(self):
        """Check Parameters."""

        self._buffer_dir = self.get_parameter("buffer_dir").get_parameter_value().string_value
        self.get_logger().info(f"[TrellisServer] Found 'buffer_dir': {self._buffer_dir}")

        self._tp_conversion_service = self.get_parameter("conversion_service").get_parameter_value().string_value
        self.get_logger().info(f"[TrellisServer] Found 'conversion_service': {self._tp_conversion_service}")

        self._conversion_model = self.get_parameter("conversion_model").get_parameter_value().string_value
        self.get_logger().info(f"[TrellisServer] Found 'conversion_model': {self._conversion_model}")

        self._qsize = self.get_parameter("qsize").get_parameter_value().integer_value
        self.get_logger().info(f"[TrellisServer] Found 'qsize': {self._qsize}")

        return True

    def initialize(self):

        if not self.__init_parameters__():
            return False

        try:
            with self._pipeline_mutex:
                self._pipeline = TrellisImageTo3DPipeline.from_pretrained(self._conversion_model)
        except Exception as err:
            self.get_logger().error(f"[TrellisServer] {err}")
            return False

        # dynamic reconfigure
        self.add_on_set_parameters_callback(self.__params_cb__)

        self._trellis_srv = self.create_service(Synthesize3DAsset, self._tp_conversion_service, self.trellis_cb, callback_group=self._process_callback_group)

        self.get_logger().info("[TrellisServer] Initialization Success!!")

        return True

    def teardown(self):
        """Teardown node."""
        self.destroy_service(self._trellis_srv)
        self.destroy_node()

    def trellis_cb(self, req, rep):
        """
        Service Callback
        """

        rep.success = True
        try:
            self.get_logger().info(f"Generating [{req.output_file}]")
            self.generate_asset(req.input_file, req.random_seed, req.simplify, req.texture_size)
            rep.message = f"Generated 3D asset from {req.input_file}!"
        except (Exception, FileNotFoundError) as err:
            rep.success = False
            rep.message = f"{err} | {traceback.format_exc()}"

        return rep

    def generate_asset(self, input_image, seed_value, simplify_value, texture_size_value):
        """
        Apply a conversion model to generate speach in the target sound font.
        """
        ret = True

        try:
            with self._pipeline_mutex:
                outputs = self._pipeline.run(
                    input_image,
                    seed=seed_value,
                    # Optional parameters
                    # sparse_structure_sampler_params={
                    #     "steps": 12,
                    #     "cfg_strength": 7.5,
                    # },
                    # slat_sampler_params={
                    #     "steps": 12,
                    #     "cfg_strength": 3,
                    # },
                )
                # outputs is a dictionary containing generated 3D assets in different formats:
                # - outputs['gaussian']: a list of 3D Gaussians
                # - outputs['radiance_field']: a list of radiance fields
                # - outputs['mesh']: a list of meshes
                
                # Render the outputs
                video = render_utils.render_video(outputs['gaussian'][0])['color']
                imageio.mimsave("sample_gs.mp4", video, fps=30)
                video = render_utils.render_video(outputs['radiance_field'][0])['color']
                imageio.mimsave("sample_rf.mp4", video, fps=30)
                video = render_utils.render_video(outputs['mesh'][0])['normal']
                imageio.mimsave("sample_mesh.mp4", video, fps=30)
                
                # GLB files can be extracted from the outputs
                glb = postprocessing_utils.to_glb(
                    outputs['gaussian'][0],
                    outputs['mesh'][0],
                    # Optional parameters
                    simplify=simplify_value,          # Ratio of triangles to remove in the simplification process
                    texture_size=texture_size_value,      # Size of the texture used for the GLB
                )
                glb.export("sample.glb")
                
                # Save Gaussians as PLY files
                outputs['gaussian'][0].save_ply("sample.ply")

        except Exception as err:
            self.get_logger().warn(f"[TrellisServer] Error: {err}")
            ret = False

        return ret

if __name__ == "__main__":

    s = TrellisServer()
