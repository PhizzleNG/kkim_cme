#!/bin/bash

set -o errexit

DOORS=$(rosservice list | sed -n '/door_/s/\(.*door_[0-9]*\)_.*/\1/p')
#if [ $# -eq 0 ] ; then
#	DOORS=$(echo "${DOORS}" | )
#fi

while true ; do
	{
	for DOOR in ${DOORS} ; do
		rosservice call ${DOOR}/open
		sleep 1
		rosservice call ${DOOR}/close
	done
	} || break
done
