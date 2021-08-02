#kill bagstart.sh process
pkill -9 -f bagstart

rosservice call /setup_camera_stream 1 0
umount /home/dji/flash