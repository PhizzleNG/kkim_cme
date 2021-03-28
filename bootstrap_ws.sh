#!/bin/bash
# Bootstrap cme_ws
# TODO: source /opt/ros setup.sh?

set -o errexit
set -o nounset
set -o pipefail

SCRIPT_ROOT="$(dirname "$(readlink -f "$0")")"

info() { echo "$(tput bold)[INFO] $*$(tput sgr0)" >&2 ; }
warn() { echo "$(tput bold)$(tput setaf 3)[WARN] $*$(tput sgr0)" >&2 ; }
error() { echo "$(tput bold)$(tput setaf 1)[ERROR] $*$(tput sgr0)" >&2 ; }
fatal() { error "$*" ; exit 1 ; }

CME_PKG="kkim_cme"
WORKSPACE="${HOME}/cme_ws"

if [ "$(basename "$(realpath "${SCRIPT_ROOT}/../CMakeLists.txt")")" = "toplevel.cmake" ] ; then
	WORKSPACE="$(realpath "${SCRIPT_ROOT}/../..")"
fi

kinova_dependencies() {
	info "Resolving kinova dependencies"
	which conan || python3 -m pip install --user conan
	conan config set general.revisions_enabled=1
	if ! conan profile show default >/dev/null 2>&1 ; then
		conan profile new default --detect > /dev/null
	fi
	conan profile update settings.compiler.libcxx=libstdc++11 default
}

create_cme_ws() {
	mkdir -p ${WORKSPACE}/src
	cd ${WORKSPACE}/src
	[ ! -e "${WORKSPACE}/src/CMakeLists.txt" ] && catkin_init_workspace
	mkdir -p ${WORKSPACE}/src
}

build_ws() {
	cd "${WORKSPACE}"
	rosdep update -q -y
	# NOTE: The first rosdep install may fail pending sources build in catkin_make
	rosdep install --from-paths src --ignore-src -y -q >/dev/null 2>&1 || true
	catkin_make -j$(nproc) >/dev/null 2>&1 || error "catkin_make failed in \"${WORKSPACE}\"!"
	. "${WORKSPACE}/devel/setup.bash"
	rosdep install --from-paths src --ignore-src -y -q || warn "rosdep install failed! Builds may be broken."
}

bootstrap_kinetic() {
	# TODO: Check git branch of cme repo and compare with ros-distro
	local _vcsfile="${SCRIPT_ROOT}/.vcsinstall"

	info "Setting up CME for kinetic in \"${WORKSPACE}\""
	sleep .5

	info "Installing dependencies"
	# Dev dependencies
	sudo apt install --quiet --yes python3 python3-pip
	python3 -m pip install --quiet --user vcstool

	# ROS dependencies
	sudo apt install --quiet --yes ros-kinetic-moveit ros-kinetic-joint-state-publisher-gui ros-kinetic-teleop-twist-keyboard

	kinova_dependencies

	info "Setting up workspace"
	create_cme_ws
	cd ${WORKSPACE}/src

	info "Pulling third-party dependencies"
	vcs import --recursive <${_vcsfile}

	if [ ! -d "${WORKSPACE}/src/${CME_PKG}" ] ; then
		if git --git-dir="${SCRIPT_ROOT}/.git" rev-parse --is-inside-work-tree 2>/dev/null ; then	
			cp -r "${SCRIPT_ROOT}" "${WORKSPACE}/src"
		else
			cd "${WORKSPACE}/src"
			git clone git@github.com:MyNameIsCosmo/kkim_cme.git
		fi
	fi

	info "Building workspace"
	build_ws
}

[ "$(ls -1 /opt/ros | wc -l)" -eq 1 ] && ROS_DISTRO=${ROS_DISTRO:-$(ls /opt/ros)}

case ${ROS_DISTRO} in
	kinetic) bootstrap_kinetic ;;
	*) fatal "\"${ROS_DISTRO}\" is not supported!" ;;
esac

info "Done!"
