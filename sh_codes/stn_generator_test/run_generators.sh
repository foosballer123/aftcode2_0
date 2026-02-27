#!/bin/bash

# 1. Validate the number of arguments (between 1 and 10)
if [ "$#" -lt 1 ] || [ "$#" -gt 10 ]; then
    echo "Error: You must provide between 1 and 10 arguments."
    exit 1
fi

# 2. Validate that each argument is an integer between 2 and 100
for arg in "$@"; do
    if ! [[ "$arg" =~ ^[0-9]+$ ]] || [ "$arg" -lt 2 ] || [ "$arg" -gt 100 ]; then
        echo "Error: Argument '$arg' is out of range (2-100) or not an integer."
        exit 1
    fi
done

# 3. Find ALL directories starting with 'test' followed by any integer
# This matches test1, test2, test999, test0, etc.
dirs=$(find . -maxdepth 1 -type d -name "test[0-9]*")

if [ -z "$dirs" ]; then
    echo "No 'testY' directories found."
    exit 0
fi

for dir in $dirs; do
    script_path="$dir/generate_files.sh"

    # 4. Check if the specific script exists in this folder
    if [ -f "$script_path" ]; then
        echo "--- Executing in $dir ---"
        
        # Move into the directory, run the script with all arguments, then come back
        (
            cd "$dir" || exit
            # Ensure the script is executable
            chmod +x generate_files.sh
            ./generate_files.sh "$@"
        )
    else
        echo "Skipping $dir: generate_files.sh not found."
    fi
done

echo "All tasks complete."
