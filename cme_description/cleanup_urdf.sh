#!/bin/bash

set -o errexit
set -o nounset
set -o pipefail

FILE=${1:-}

if [ ! -f "${FILE}" ] ; then
	echo "\"${FILE}\" doesn't exist!." >&2
	exit 1
fi

# Ensure arguments are spaced properly
sed -i -r 's/="\s*([0-9.-]+)\s+([0-9.-]+)\s+([0-9.-]+)\s*"/="\1 \2 \3"/g' "${FILE}"

# Set mass values above 0
sed -i '/mass/s/value="0"/value="1"/' "${FILE}"
