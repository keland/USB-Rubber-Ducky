#!/bin/sh

EXIT=0
if lsusb | awk '$6=="03eb:2ff6"{e=1}END{exit e}'; then
    echo "Is your Ducky connected and in DFU mode?  I don't see it.  Try"
    echo "unplugging it, then holding down the button while plugging it back in."
    EXIT=1
fi

FILE=${1:-dump.bin}		# Where to put our dump.
if [ -f "$FILE" ]; then
    echo "Output file already exists.  Remove it or specify another."
    EXIT=1
fi

if ! which dfu-programmer &>/dev/null; then
    echo "dfu-programmer not found.  Go install it and try again."
    EXIT=1
fi
test $EXIT -eq 1 && exit 1	# Get all the errors at once, then exit

sudo dfu-programmer at32uc3b1256 dump >$FILE
echo Dump complete.  Resetting your Ducky...
sudo dfu-programmer at32uc3b1256 reset
