#!/bin/bash
# Bootstrap cme_ws
# TODO: source /opt/ros setup.sh?

set -o errexit
set -o nounset
set -o pipefail

SCRIPT="$(readlink -f "${0}")"
SCRIPT_ROOT="$(dirname "${SCRIPT}")"

LOG_DIR=$(mktemp -d /tmp/CME_XXXXXX)

declare -i WARNS=0
declare -i ERRORS=0

log() { echo "$*" ; echo "$*" > ${LOG_DIR}/bootstrap.log ; }
info() { log "$(tput bold)[INFO] $*$(tput sgr0)" ; }
warn() { log "$(tput bold)$(tput setaf 3)[WARN] $*$(tput sgr0)" >&2 ; WARNS+=1 ; }
error() { log "$(tput bold)$(tput setaf 1)[ERROR] $*$(tput sgr0)" >&2 ; ERRORS+=1 ; }
print_exit() { info "$(tput setaf 3)Additional log files may be availble in \"${LOG_DIR}\"" ; }
fatal() { error "$*" ; print_exit ; exit 1 ; }

CME_PKG="${CME_PKG:-}"
WORKSPACE="${HOME}/cme_ws"

DISABLE_KINOVA=${DISABLE_KINOVA:-0}

[ -z "${CME_PKG}" ] && CME_PKG="$(basename $(dirname "${SCRIPT}"))"

if [ "$EUID" -eq 0 ] && [ -z "${FORCE_ROOT:-}" ] ; then
	fatal "This script should not be run as root."
fi

if [ "$(basename "$(realpath "${SCRIPT_ROOT}/../CMakeLists.txt")")" = "toplevel.cmake" ] ; then
	WORKSPACE="$(realpath "${SCRIPT_ROOT}/../..")"
else
	fatal "\"${CME_PKG}\" is not in a catkin_ws! Please ensure you have run \`catkin_init_workspace\` in \"$(realpath "${SCRIPT_ROOT}")\""
fi

info "Logging to: \"${LOG_DIR}\""

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
		warn "\"${SCRIPT_ROOT}\" is not a git repo!"
	fi
	error "Unable to update CME! You may be out of date."
	sleep 2
	return 1
}

kinova_dependencies() {
	info "Resolving kinova dependencies"
	# TODO: Use a virtualenv instead, set ros env-hook in kinova pkgs?
	if ! which conan >${LOG_DIR}/which_conan.log 2>&1; then
		if ! /usr/bin/python3.5 -m pip install --quiet --user conan ; then
			error "Unable to install conan!"
			return 1
		fi
	fi
	conan config set general.revisions_enabled=1 > /dev/null
	if ! conan profile show default >${LOG_DIR}/conan_profile.log 2>&1 ; then
		conan profile new default --detect >${LOG_DIR}/conan_new_profile.log
	fi
	if ! conan profile update settings.compiler.libcxx=libstdc++11 default > ${LOG_DIR}/conan_profile_update.log ; then
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
	rosdep update -y >"${LOG_DIR}/rosdep_update_1.log"
	info "Resolving workspace dependencies"
	# NOTE: The first rosdep install may fail pending sources build in catkin_make
	rosdep install --from-paths src --ignore-src --skip-keys="$(cat "${WORKSPACE}"/.rosdep_skip_keys 2>/dev/null || true)" -y -q >"${LOG_DIR}/rosdep_install.log" 2>&1 || true
	info "Building workspace"
	catkin_make -j$(nproc) >"${LOG_DIR}/catkin_make.log" 2>&1 || error "catkin_make failed in \"${WORKSPACE}\"!"
	. "${WORKSPACE}/devel/setup.bash"
	rosdep install --from-paths src --ignore-src --skip-keys="$(cat "${WORKSPACE}"/.rosdep_skip_keys 2>/dev/null || true)" -y -qqq || warn "rosdep install failed! Builds may be broken."
}

disable_kinova_build() {
	find "${WORKSPACE}"/src/kinova-ros -maxdepth 1 -type d -not -ipath '*/.*' -exec touch {}/CATKIN_IGNORE \;
	find "${WORKSPACE}"/src/ros_kortex -maxdepth 1 -type d -not -ipath '*/.*' -exec touch {}/CATKIN_IGNORE \;
	printf "%s" " kinova_description kortex_driver" >> "${WORKSPACE}/.rosdep_skip_keys"
}

enable_kinova_build() {
	find "${WORKSPACE}"/src/kinova-ros -maxdepth 2 -type f -name 'CATKIN_IGNORE' -not -ipath '*/.*' -delete || true
	find "${WORKSPACE}"/src/ros_kortex -maxdepth 2 -type f -name 'CATKIN_IGNORE' -not -ipath '*/.*' -delete || true
	if [ -f "${WORKSPACE}"/.rosdep_skip_keys ] ; then
		sed -i 's/\s*kinova_description//' "${WORKSPACE}"/.rosdep_skip_keys
		sed -i 's/\s*kortex_driver//' "${WORKSPACE}"/.rosdep_skip_keys
	fi
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

	info "Setting up workspace"
	create_cme_ws
	cd ${WORKSPACE}/src

	info "Pulling third-party dependencies"
	vcs import --recursive >"${LOG_DIR}/vcs.log" <${_vcsfile} || fatal "Unable to pull external dependencies!"

	info "Installing dependencies"
	# Dev dependencies
	sudo apt-get install -qq --yes python3 python3-pip
	/usr/bin/python3.5 -m pip install --quiet --user vcstool

	# ROS dependencies
	sudo apt-get install -qq --yes ros-kinetic-moveit ros-kinetic-joint-state-publisher-gui ros-kinetic-teleop-twist-keyboard
	sudo apt-get install -qq --yes ros-kinetic-prbt-moveit-config ros-kinetic-panda-moveit-config
	sudo apt-get install -qq --yes ros-kinetic-jackal-simulator ros-kinetic-jackal-desktop ros-kinetic-jackal-navigation

	if [ ${DISABLE_KINOVA} -eq 1 ]; then
		warn "Disabling kinova build!"
		disable_kinova_build
	elif kinova_dependencies ; then
		# Ensure kinova deps are built with catkin if kinova_dependencies is successful
		enable_kinova_build
	else
		error "Unable to resolve kinova dependencies!"
		error "Marking kinova packages to not build in catkin_make."
		disable_kinova_build
	fi

	build_ws
}

[ "$(ls -1 /opt/ros | wc -l)" -eq 1 ] && ROS_DISTRO=${ROS_DISTRO:-$(ls /opt/ros)}

env > "${LOG_DIR}/env.log"

case ${ROS_DISTRO} in
	kinetic) bootstrap_kinetic ;;
	*) fatal "\"${ROS_DISTRO}\" is not supported!" ;;
esac

if [ ${WARNS} -gt 0 ] || [ ${ERRORS} -gt 0 ] ; then
	print_exit
fi

info "Done!"
