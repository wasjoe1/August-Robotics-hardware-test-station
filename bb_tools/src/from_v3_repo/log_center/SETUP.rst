Use system service to detect DBholder mounted
**This requires our ``DBholder`` USB storage.**

1. Set new ``sync_log.rules`` in ``/etc/udev/rules.d``. Make sure the **username** is changed
   for target PCs. Make sure the **/dev/sdb1** is correct for inserted USB (sometimes it's ``/dev/sda1``)
::

   ACTION=="add", SUBSYSTEM=="usb", ATTR{idVendor}=="0951", ATTR{idProduct}=="1666", RUN+="/home/<USER>/catkin_ws/src/boothbot/log_center/usb.sh <USER> /dev/sdb1"


2. Re-load rules ``sudo udevadm control --reload-rules && sudo udevadm trigger``

3. This script will be run as ``root`` user.
