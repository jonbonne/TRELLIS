#!/bin/bash

# Ensure the source and destination directories are provided
if [ "$#" -ne 2 ]; then
    echo "Usage: $0 <source_project_root> <destination_root>"
    exit 1
fi

SOURCE_ROOT=$1
DEST_ROOT=$2

# Find all package.xml files in the source directory and copy them
find "$SOURCE_ROOT" -name 'package.xml' -type f | while read -r file; do
    # Get the directory containing the package.xml
    dir=$(dirname "$file")
    
    # Compute the relative path from the source root
    relative_path=${dir#"$SOURCE_ROOT"}
    
    # Create the same directory structure in the destination
    mkdir -p "$DEST_ROOT$relative_path"
    
    # Copy the package.xml file to the corresponding directory in the destination
    cp "$file" "$DEST_ROOT$relative_path/"
done

echo "All package.xml files have been copied to $DEST_ROOT"
