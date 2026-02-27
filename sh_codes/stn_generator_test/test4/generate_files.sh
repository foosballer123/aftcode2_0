#!/bin/bash

# This placeholder will be replaced by the meta_generator
SCHEME_TEMPLATE="st4_p0_hX_bCR_MV_HL"
TEMPLATE_FILE="../template.sh"

if [ ! -f "$TEMPLATE_FILE" ]; then
    echo "Error: ../template.sh not found."
    exit 1
fi

if [ $# -lt 1 ] || [ $# -gt 10 ]; then
    echo "Usage: $0 [1 to 10 integers between 2-100]"
    exit 1
fi

FILES_TO_CREATE=()

for val in "$@"; do
    if [[ "$val" =~ ^[0-9]+$ ]] && [ "$val" -ge 2 ] && [ "$val" -le 100 ]; then
        # Replace 'hX' from the scheme with 'h' + the user input
        NEW_FILENAME=$(echo "$SCHEME_TEMPLATE" | sed "s/hX/h${val}/")
        
        if [ -f "$NEW_FILENAME" ]; then
            echo "Note: '$NEW_FILENAME' already exists. Skipping."
        else
            FILES_TO_CREATE+=("$NEW_FILENAME")
        fi
    else
        echo "Warning: '$val' is invalid (must be 2-100). Skipping."
    fi
done

if [ ${#FILES_TO_CREATE[@]} -eq 0 ]; then
    echo "No new files to create."
    exit 0
fi

echo -e "\nProposed files to create in $(basename "$PWD"):"
for f in "${FILES_TO_CREATE[@]}"; do echo "  - $f"; done

read -p "Proceed? (y/n): " confirm
if [[ "$confirm" =~ ^[yY](es)?$ ]]; then
    for f in "${FILES_TO_CREATE[@]}"; do
        cp "$TEMPLATE_FILE" "$f"
        echo "Created: $f"
    done
else
    echo "Aborted."
fi
