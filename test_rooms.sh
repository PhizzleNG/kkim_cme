#!/bin/bash

set -o errexit

DOORS=$(rosservice list | sed -nr '/door_/s/(.*door_[0-9]+)_.*/\1/p' | uniq)
LIGHTS=$(rosservice list | sed -nr '/light_/s/(.*light_[0-9]+).*/\1/p' | uniq)

ROOMS=$(echo "${DOORS}" | cut -d'_' -f2)

while true ; do
	{
	for ROOM in ${ROOMS} ; do
		DOOR=$(echo "${DOORS}" | grep "door_${ROOM}")
		LIGHT=$(echo "${LIGHTS}" | grep "light_${ROOM}")
		[ -n "${DOOR}" ] && rosservice call ${DOOR}/open
		[ -n "${LIGHT}" ] && rosservice call ${LIGHT}/on
		sleep .5
		[ -n "${LIGHT}" ] && rosservice call ${LIGHT}/off
		[ -n "${DOOR}" ] && rosservice call ${DOOR}/close
	done
	} || break
done

