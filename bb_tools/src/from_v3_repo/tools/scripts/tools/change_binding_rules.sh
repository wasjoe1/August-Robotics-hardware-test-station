#!/usr/bin/env bash

exec < /dev/tty

FILE=/etc/udev/rules.d/bb_j1900.rules
echo `grep sonars $FILE`
echo `grep clinometer $FILE`
echo `grep mobile_base $FILE`
echo `grep io_module $FILE`
read -p 'Need to modify udev rules? (y/n)' yn

CMD="sudo nano /etc/udev/rules.d/bb_j1900.rules"

if [[ "$yn" == "y" ]]; then
    ${CMD}
    read -p 'Save and reload udev rules? (y/n)' yn2
    if [[ "$yn2" == "y" ]]; then
        sudo udevadm control --reload-rules
        sudo udevadm trigger
        echo 'Reload DONE!'
    fi
fi
