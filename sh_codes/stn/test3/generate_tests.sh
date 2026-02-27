#!/bin/bash

# Path to your template file in the parent directory
TEMPLATE="../template.sh"

# Check if the template exists before proceeding
if [ ! -f "$TEMPLATE" ]; then
    echo "Error: Template file '$TEMPLATE' not found in the parent directory."
    exit 1
fi

# Check if at least one argument was provided
if [ $# -eq 0 ]; then
    echo "Usage: $0 int1 [int2 ... int10]"
    exit 1
fi

# Limit to 10 arguments
if [ $# -gt 10 ]; then
    echo "Error: Please provide a maximum of 10 integer arguments."
    exit 1
fi

# Iterate through each argument provided
for val in "$@"; do
    # Validate that the input is an integer between 2 and 100
    if [[ "$val" =~ ^[0-9]+$ ]] && [ "$val" -ge 2 ] && [ "$val" -le 100 ]; then
        
        # Construct the new filename
        NEW_FILENAME="st3_p120_h${val}_bBR_LV_DL"
        
        # Check if the file already exists
        if [ -f "$NEW_FILENAME" ]; then
            echo "Skipping: '$NEW_FILENAME' already exists."
        else
            # Copy the template to the new filename
            cp "$TEMPLATE" "$NEW_FILENAME"
            echo "Created: $NEW_FILENAME"
        fi
    else
        echo "Error: '$val' is not a valid integer between 2 and 100. Skipping."
    fi
done
