ip="$(ifconfig | grep  192.168.1 | cut -d ':' -f 2 |  cut -d ' ' -f 1)"
echo $ip
echo $1
echo "gst-launch v4l2src device=/dev/video1 ! videorate ! 'video/x-raw-yuv,framerate=15/1' ! videoscale ! video/x-raw-yuv, width=544, height=304 ! example tcp_port=2002 threshtune=99 ! dspmp4venc ! rtpmp4vpay config-interval=2 ! udpsink host=$ip port=5000" >> $1/initvideoall.sh
