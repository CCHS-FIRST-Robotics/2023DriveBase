#! /bin/sh

TARGETIP="10.32.5.5"
PORT=5000
DEVICE="/dev/videolifecam0"
WIDTH=640
HEIGHT=480
FRAMERATE=30
IFRAMEINT=30

gst-launch-1.0 v4l2src device=$DEVICE ! \
video/x-raw,format=YUY2 ! nvvidconv  ! \
"video/x-raw(memory:NVMM), \
width=(int)$WIDTH, \
height=(int)$HEIGHT, \
format=(string)NV12, \
framerate=(fraction)$FRAMERATE/1" ! \
nvv4l2h265enc maxperf-enable=1 bitrate=2000000 \
iframeinterval=$IFRAMEINT preset-level=1 control-rate=1 ! \
h265parse ! rtph265pay config-interval=1 ! \
udpsink host=$TARGETIP \
port=$PORT \
sync=false async=false
