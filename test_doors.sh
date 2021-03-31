#!/bin/bash

doors=$(rosservice list | sed -n '/door_/s/\(.*door_[0-9]*\)_.*/\1/p')

while true ; do
	{
	for door in ${doors} ; do
		rosservice call ${door}/open
		sleep 1
		rosservice call ${door}/close
	done
	} || break
done
