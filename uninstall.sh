#!/bin/bash
# Script to unbind the driver from any connected microlites, so I don't have
# to repeatedly disconnect and reconnect the USB each time I make a change.
pushd /sys/bus/usb/devices
for F in */uevent; do
	if $(grep -F "DRIVER=snd-microlite" "${F}" > /dev/null); then
		echo ${F%/*} > /sys/bus/usb/drivers/snd-microlite/unbind
	fi
done
popd
