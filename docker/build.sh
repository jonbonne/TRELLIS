#!/bin/bash

set -e

if [[ -z "${OS_ARCH}" ]]; then
  echo "Warning: OS_ARCH is not set or empty. Defaulting to 'amd64' image build."
  OS_ARCH="amd64"
fi

# use git hash for version
CHANGELOG_VERSION=$(git rev-parse --short HEAD)
TAG=${CHANGELOG_VERSION}

AUX_IMAGE=${OS_ARCH}/trellis_ros
BASE_IMAGE="nvidia/cuda:12.2.0-devel-ubuntu22.04"
ROS_DISTRO="rolling"
echo -en "Building ${IMAGE}:${ROS_DISTRO}-${TAG}\n"
echo -en "- base-image: ${BASE_IMAGE}\n"
echo -en "- platform: linux/${OS_ARCH}\n"
docker buildx build --load \
    --build-arg BASE_IMAGE=${BASE_IMAGE} \
    --build-arg OS_ARCH=${OS_ARCH} \
    --build-arg ROS_DISTRO=${ROS_DISTRO} \
    --target trellis_main \
    -t ${AUX_IMAGE}:${ROS_DISTRO}-${TAG} ..
docker tag ${AUX_IMAGE}:${ROS_DISTRO}-${TAG} ${AUX_IMAGE}:${ROS_DISTRO}-latest
