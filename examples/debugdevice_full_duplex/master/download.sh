#!/bin/bash -e

DEVS=$(lsusb|grep 04b4:8619|sed 's/:.*//;s/Bus //;s/Device //;s/ /\//')

if [ -z "$1" ]; then
    echo "$0: usage: $0 <file>"
    exit 1;
fi

for dev in $DEVS;do
    echo "Downloading $1 to $dev"
    sudo /sbin/fxload -v -D /dev/bus/usb/$dev -t fx2lp -I $1 -c 0x01 -s /home/administrator/Vend_Ax.hex
done

exit 0
