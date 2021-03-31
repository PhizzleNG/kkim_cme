#!/bin/bash
# Bootstrap cme_ws
# TODO: source /opt/ros setup.sh?

set -o errexit
set -o nounset
set -o pipefail

SCRIPT="$(readlink -f "${0}")"
SCRIPT_ROOT="$(dirname "${SCRIPT}")"

info() { echo "$(tput bold)[INFO] $*$(tput sgr0)" >&2 ; }
warn() { echo "$(tput bold)$(tput setaf 3)[WARN] $*$(tput sgr0)" >&2 ; }
error() { echo "$(tput bold)$(tput setaf 1)[ERROR] $*$(tput sgr0)" >&2 ; }
fatal() { error "$*" ; exit 1 ; }

CME_PKG="kkim_cme"
WORKSPACE="${HOME}/cme_ws"

if [ "$EUID" -eq 0 ] && [ -z "${FORCE_ROOT:-}" ] ; then
	fatal "This script should not be run as root."
fi

if [ "$(basename "$(realpath "${SCRIPT_ROOT}/../CMakeLists.txt")")" = "toplevel.cmake" ] ; then
	WORKSPACE="$(realpath "${SCRIPT_ROOT}/../..")"
else
	fatal "\"${CME_PKG}\" is not in a catkin_ws! Please ensure you have run \`catkin_init_workspace\` in \"$(realpath "${SCRIPT_ROOT}")\""
fi

update_cme() {
	cd "${SCRIPT_ROOT}"
	if [ -d ".git" ] ; then
		git fetch
		# check if head matches with tracking branch (if any)
		[ "$(git rev-parse HEAD)" == "$(git rev-parse @{u})" ] && return 1
		if ! git diff --quiet ; then
			warn "Local changes detected in repo!"
			return 1
		fi
		if git status | grep -q "branch is ahead" ; then
			warn "Local branch is ahead of upstream!"
			return 1
		fi
		git pull && return 0
	else
		warn "\"${SCRPIT_ROOT}\" is not a git repo!"
	fi
	error "Unable to update CME! You may be out of date."
	sleep 2
	return 1
}

kinova_dependencies() {
	info "Resolving kinova dependencies"
	if ! which conan > /dev/null 2>&1; then
		if ! /usr/bin/python3.5 -m pip install --quiet --user conan ; then
			error "Unable to install conan!"
			return 1
		fi
	fi
	conan config set general.revisions_enabled=1 > /dev/null
	if ! conan profile show default >/dev/null 2>&1 ; then
		conan profile new default --detect > /dev/null
	fi
	if ! conan profile update settings.compiler.libcxx=libstdc++11 default > /dev/null ; then
		error "Unable to update conan profile!"
		return 1
	fi
	return 0
}

create_cme_ws() {
	mkdir -p ${WORKSPACE}/src
	cd ${WORKSPACE}/src
	[ ! -e "${WORKSPACE}/src/CMakeLists.txt" ] && catkin_init_workspace
	mkdir -p ${WORKSPACE}/src
}

build_ws() {
	cd "${WORKSPACE}"
	info "Updating rosdep"
	rosdep update -y >/dev/null
	info "Resolving workspace dependencies"
	# NOTE: The first rosdep install may fail pending sources build in catkin_make
	rosdep install --from-paths src --ignore-src --skip-keys="$(cat "${WORKSPACE}"/.rosdep_skip_keys 2>/dev/null || true)" -y -q >/dev/null 2>&1 || true
	info "Building workspace"
	catkin_make -j$(nproc) >/dev/null 2>&1 || error "catkin_make failed in \"${WORKSPACE}\"!"
	. "${WORKSPACE}/devel/setup.bash"
	rosdep install --from-paths src --ignore-src --skip-keys="$(cat "${WORKSPACE}"/.rosdep_skip_keys 2>/dev/null || true)" -y -qqq || warn "rosdep install failed! Builds may be broken."
}

bootstrap_kinetic() {
	# TODO: Check git branch of cme repo and compare with ros-distro
	local _vcsfile="${SCRIPT_ROOT}/.vcsinstall"

	if update_cme ; then
		# Restart script
		exec "${SCRIPT}" "${@}"
	fi

	info "Setting up CME for kinetic in \"${WORKSPACE}\""
	sleep .5

	info "Installing dependencies"
	# Dev dependencies
	sudo apt-get install -qq --yes python3 python3-pip
	/usr/bin/python3.5 -m pip install --quiet --user vcstool

	# ROS dependencies
	sudo apt-get install -qq --yes ros-kinetic-moveit ros-kinetic-joint-state-publisher-gui ros-kinetic-teleop-twist-keyboard

	if kinova_dependencies ; then
		# Ensure kinova deps are built with catkin
		find "${WORKSPACE}"/src/kinova-ros -maxdepth 2 -type f -name 'CATKIN_IGNORE' -not -ipath '*/.*' -delete || true
		find "${WORKSPACE}"/src/ros_kortex -maxdepth 2 -type f -name 'CATKIN_IGNORE' -not -ipath '*/.*' -delete || true
		if [ -f "${WORKSPACE}"/.rosdep_skip_keys ] ; then
			sed -i 's/\s*kinova_description//' "${WORKSPACE}"/.rosdep_skip_keys
			sed -i 's/\s*kortex_driver//' "${WORKSPACE}"/.rosdep_skip_keys
		fi
	else
		error "Unable to resolve kinova dependencies!"
		error "Marking kinova packages to not build in catkin_make."
		find "${WORKSPACE}"/src/kinova-ros -maxdepth 1 -type d -not -ipath '*/.*' -exec touch {}/CATKIN_IGNORE \;
		find "${WORKSPACE}"/src/ros_kortex -maxdepth 1 -type d -not -ipath '*/.*' -exec touch {}/CATKIN_IGNORE \;
		printf "%s" " kinova_description kortex_driver" >> "${WORKSPACE}/.rosdep_skip_keys"
	fi

	info "Setting up workspace"
	create_cme_ws
	cd ${WORKSPACE}/src

	info "Pulling third-party dependencies"
	vcs import --recursive >/dev/null <${_vcsfile}

	build_ws
}

[ "$(ls -1 /opt/ros | wc -l)" -eq 1 ] && ROS_DISTRO=${ROS_DISTRO:-$(ls /opt/ros)}

case ${ROS_DISTRO} in
	kinetic) bootstrap_kinetic ;;
	*) fatal "\"${ROS_DISTRO}\" is not supported!" ;;
esac

info "Done!"
