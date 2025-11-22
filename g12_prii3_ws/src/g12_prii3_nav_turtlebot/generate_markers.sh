#!/bin/bash

# Create models directory if it doesn't exist
mkdir -p models

# Clean up previous runs
rm -rf temp_gazebo_models temp_images temp_generated_models temp_check_repo

# Create a temporary directory for cloning
mkdir -p temp_gazebo_models
mkdir -p temp_images

# Clone the repository
git clone https://github.com/mikaelarguedas/gazebo_models.git temp_gazebo_models

# Prepare images for IDs 1, 2, 3
cp temp_gazebo_models/ar_tags/images/Marker1.png temp_images/
cp temp_gazebo_models/ar_tags/images/Marker2.png temp_images/
cp temp_gazebo_models/ar_tags/images/Marker3.png temp_images/

# Generate markers
mkdir -p temp_generated_models

echo "Running generate_markers_model.py..."
python3 temp_gazebo_models/ar_tags/scripts/generate_markers_model.py -i $PWD/temp_images -g $PWD/temp_generated_models

# Fix texture locations: Move PNGs from materials/textures to meshes/
# The DAE files expect the texture in the same directory
for marker_dir in temp_generated_models/*; do
    if [ -d "$marker_dir" ]; then
        echo "Fixing textures for $marker_dir"
        mv "$marker_dir/materials/textures/"*.png "$marker_dir/meshes/"
        # Remove the now empty materials directory structure
        rm -rf "$marker_dir/materials"
    fi
done

# Move and rename the folders
# The script generates folders named marker1, marker2, etc. (lowercase)
if [ -d "temp_generated_models/marker1" ]; then
    rm -rf models/ar_marker_1
    mv temp_generated_models/marker1 models/ar_marker_1
fi

if [ -d "temp_generated_models/marker2" ]; then
    rm -rf models/ar_marker_2
    mv temp_generated_models/marker2 models/ar_marker_2
fi

if [ -d "temp_generated_models/marker3" ]; then
    rm -rf models/ar_marker_3
    mv temp_generated_models/marker3 models/ar_marker_3
fi

# Clean up
rm -rf temp_gazebo_models
rm -rf temp_images
rm -rf temp_generated_models

echo "Markers generated and moved to models/ directory as ar_marker_1, ar_marker_2, ar_marker_3."
