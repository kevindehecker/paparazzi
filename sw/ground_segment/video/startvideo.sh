gst-launch-0.10 udpsrc uri=udp://0.0.0.0:5000  caps = 'application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)MP4V-ES, payload=(int)96' ! rtpmp4vdepay ! ffdec_mpeg4 ! ffmpegcolorspace ! tee name=split ! ximagesink split. ! queue ! ffmpegcolorspace ! ffenc_mpeg4 ! avimux ! filesink location=video.avi
