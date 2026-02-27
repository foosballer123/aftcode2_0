#!/bin/bash

SCHEMES_FILE="schemes.txt"
GEN_TEMPLATE="generator_template.sh"

if [[ ! -f "$SCHEMES_FILE" ]] || [[ ! -f "$GEN_TEMPLATE" ]]; then
    echo "Error: Ensure both $SCHEMES_FILE and $GEN_TEMPLATE exist."
    exit 1
fi

while IFS= read -r line || [[ -n "$line" ]]; do
    [[ -z "$line" ]] && continue

    # Extract integer after 'st' to name the folder
    if [[ "$line" =~ st([0-9]+) ]]; then
        ST_NUM="${BASH_REMATCH[1]}"
        DIR_NAME="test${ST_NUM}"

        # Create directory if missing
        [ ! -d "$DIR_NAME" ] && mkdir "$DIR_NAME" && echo "Created directory: $DIR_NAME"

        # Define path for the new generator
        TARGET_GEN="$DIR_NAME/generate_files.sh"

        # Only create if it doesn't exist
        if [ ! -f "$TARGET_GEN" ]; then
            # Copy template and replace the placeholder with the actual line from schemes.txt
            sed "s/CHOSEN_SCHEME/$line/" "$GEN_TEMPLATE" > "$TARGET_GEN"
            chmod +x "$TARGET_GEN"
            echo "  -> Setup $TARGET_GEN"
        else
            echo "  -> $TARGET_GEN already exists. Skipping."
        fi
    fi
done < "$SCHEMES_FILE"

echo "Meta-generation complete."
