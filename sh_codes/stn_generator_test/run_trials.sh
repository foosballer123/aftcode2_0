#!/bin/bash

# Define the target directory (current directory by default)
TARGET_DIR="."

# Use 'find' to locate files
# -type f: look for files only
# -name 's*': filename must start with 's'
# -executable: ensure we only try to run files with execute permissions
find "$TARGET_DIR" -type f -name 's*' -executable | while read -r script_path; do
    echo "Now running: $script_path"
    
    # Execute the script
    # The shell waits for this to finish before the next loop iteration
    "$script_path"
    
    echo "Finished: $script_path"
    echo "--------------------------"
done

echo "All matching scripts have been processed."
