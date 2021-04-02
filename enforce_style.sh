#!/bin/bash

set -o errexit
set -o nounset
set -o pipefail

SCRIPT_ROOT="$(dirname "$(readlink -f "${0}")")"

format_xml() {
	local XML_DIR
	local XML_NAME
	XML_DIR="${1:-${SCRIPT_ROOT}}"
	XML_NAME="${2:-*.xacro}"

	XMLLINT_INDENT=$'\t'

	local FILE
	local TMP_FILE
	for FILE in find "${XML_DIR}" -iname "${XML_NAME}" ; do
		TMP_FILE=$(mktemp)
		xmllint --format "${FILE}" > "${TMP_FILE}"
		mv "${TMP_FILE}" "${FILE}"
	done
}

format_xml "${SCRIPT_ROOT}" "*.xml"
format_xml "${SCRIPT_ROOT}" "*.urdf"
format_xml "${SCRIPT_ROOT}" "*.xacro"
