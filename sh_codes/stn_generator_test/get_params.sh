#!/bin/bash

get_params() {
	# 1. Get the filename without the path or the .sh extension
	base_name=$(basename "$0" .sh)

	# 2. Split the filename into an array using '_' as the delimiter
	# This assumes the format is action_type_frequency
	IFS='_' read -r name player horizon ball <<< "$base_name"
	
	test=${name//[!0-9]/}
	y1=${player//[!0-9]/}
	h=${horizon//[!0-9]/}
}
