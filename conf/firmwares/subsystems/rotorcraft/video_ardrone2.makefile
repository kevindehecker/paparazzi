# Hey Emacs, this is a -*- makefile -*-
#
# 
#

VIDEO_CFLAGS  = -DUSE_VIDEO=0 -DUSE_VIDEO_ARDRONE2=0

VIDEO_CFLAGS += -DVIDEO_TYPE_H=\"subsystems/video/video_ardrone2.h\"
VIDEO_SRCS   += subsystems/video.c
VIDEO_SRCS   += subsystems/video/video_ardrone2.c

ap.CFLAGS += $(VIDEO_CFLAGS)
ap.srcs += $(VIDEO_SRCS)

nps.CFLAGS += $(VIDEO_CFLAGS)
nps.srcs += $(VIDEO_SRCS)

