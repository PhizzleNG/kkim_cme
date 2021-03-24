#!/bin/bash
# Bootstrap cme_ws
# TODO: source /opt/ros setup.sh?

set -o errexit
set -o nounset
set -o pipefail

SCRIPT_ROOT="$(dirname "$(readlink -f "$0")")"

error() { echo "$*" >&2 }
fatal() { error "$*" ; exit 1 }

CME_PKG="kkim_cme"
WORKSPACE="${HOME}/cme_ws"

if [ "$(basename "$(realpath "${SCRIPT_ROOT}/../CMakeLists.txt")")" = "toplevel.cmake" ] ; then
	WORKSPACE="$(realpath "${SCRIPT_ROOT}/..")"
fi

check_dependencies() {
	local _apt_deps=${1}
	local _pip_deps=${2}
	[ -n "${_apt_deps}" ] && sudo apt install ${_apt_deps}
	[ -n "${_pip_deps}" ] && pip3 install --user ${_pip_deps}
}

kinova_dependencies() {
	python3 -m pip install --user conan
	conan config set general.revisions_enabled=1
	conan profile new default --detect > /dev/null
	conan profile update settings.compiler.libcxx=libstdc++11 default
}

create_cme_ws() {
	mkdir -p ${WORKSPACE}/src
	[ ! -f ${WORKSPACE}/src/CMakeLists.txt ] && catkin_init_workspace
	cd ${WORKSPACE}/src
	mkdir -p ${WORKSPACE}/src
}

build_ws() {
	cd "${WORKSPACE}"
	rosdep update
	# NOTE: The first rosdep install may fail pending sources build in catkin_make
	rosdep install --from-patsh src --ignore-src -y || true
	catkin_make -j$(nproc)
	rosdep install --from-patsh src --ignore-src -y
}

bootstrap_kinetic() {
	# TODO: Check git branch of cme repo and compare with ros-distro
	local _vcsfile="${SCRIPT_ROOT}/.vcsinstall"

	# Dev dependencies
	sudo apt install -y python3 python3-pip
	python3 -m pip install --user vcstool

	# ROS dependencies
	sudo apt install ros-kinetic-moveit ros-kinetic-joint-state-publisher-gui

	kinova_dependencies

	create_cme_ws
	cd ${WORKSPACE}/src
	vcstool import --recursive <${_vcsfile}

	if [ ! -d "${WORKSPACE}/src/${CME_PKG}" ] ; then
		if git --git-dir="${SCRIPT_ROOT}/.git" rev-parse --is-inside-work-tree 2>/dev/null ; then	
			# copy SCRIPT_ROOT to cme_ws
			
		else
			echo "TODO: Clone git repo!"	
		fi
	fi

	build_ws
}

[ "$(ls -1 /opt/ros | wc -l)" -eq 1 ] && ROS_DISTRO=${ROS_DISTRO:-$(ls /opt/ros)}

case ${ROS_DISTRO} in
	kinetic) bootstrap_kinetic ;;
	*) fatal "\"${ROS_DISTRO}\" is not supported!" ;;
esac

echo "Done!"
