# stat=$(ifconfig | grep -i can0)
# echo stat=$stat
# if [ !"$stat" ]
if [ $(ifconfig | grep -i can0 | wc -c) -eq 0 ]
then
    ip link set can0 down
    modprobe can_raw
    modprobe can_dev
    insmod usb_8dev.ko
    ip link set can0 up type can bitrate 500000 sample-point 0.875
fi
