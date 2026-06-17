#!/bin/bash

# Fix model URIs in SDF files
# The folders are ar_marker_1, ar_marker_2, ar_marker_3
# But the SDF files contain model://marker1, model://marker2, model://marker3

cd models

for i in 1 2 3; do
    dir="ar_marker_$i"
    old_name="marker$i"
    new_name="ar_marker_$i"
    
    echo "Processing $dir..."
    
    if [ -d "$dir" ]; then
        # Find all sdf files
        find "$dir" -name "*.sdf" | while read sdf_file; do
            echo "  Updating $sdf_file"
            # Replace model://markerX with model://ar_marker_X
            sed -i "s|model://$old_name|model://$new_name|g" "$sdf_file"
        done
        
        # Also update model.config name if desired, but keeping "MarkerX" as display name is fine.
        # However, let's check if model.config references the folder name. Usually not.
    else
        echo "  Directory $dir not found!"
    fi
done

echo "URIs updated."
