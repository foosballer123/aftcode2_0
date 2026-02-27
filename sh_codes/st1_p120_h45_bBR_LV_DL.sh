#!/bin/bash

get_params() {

	# 1. Get the filename without the path or the .sh extension
	base_name=$(basename "$0" .sh)

	# 2. Split the filename into an array using '_' as the delimiter
	# This assumes the format is action_type_frequency
	IFS='_' read -r name player horizon ball <<< "$base_name"
	
}

get_params

# 3. Use your "parameters"
echo "Base Name: $base_name"
echo "Name: $name"
echo "Player:   $player"
echo "Horizon:   $horizon"
echo "Ball:   $ball"

# 4. Clean your "parameters"
echo -e "\\nBase Name: $base_name"
echo "Name (cleaned): ${name//[!0-9]/}"
echo "Player (cleaned):   ${player//[!0-9]/}"
echo "Horizon (cleaned):   ${horizon//[!0-9]/}"
echo "Ball:   ${ball}" 
