# Jetson Nano Video Streaming
Low latency h265 video streaming using Microsoft Lifecam. 
Bandwidth compression, resolution, framerate configurable in stream.sh 
Microsoft camera auto identified and mapped to /dev/videolifecam0
The video streaming service will start at boot time

## Linux Install 
#### Gstreamer 
`sudo apt install libgstrtspserver-1.0 libgstreamer1.0-dev`
#### video for linux drivers
`sudo apt install v4l-utils`

## Gstreamer windows install 
https://gstreamer.freedesktop.org/documentation/installing/on-windows.html?gi-language=c 


## Configure streaming parameters
Set target ip, width, height, etc.
Make sure to edit TARGETIP
`root/usr/local/bin/stream.sh`

After installation edit 
`/usr/local/bin/stream.sh`

## Install 
`sudo make install`

## Uninstall 
`sudo make uninstall`

## Video reception
Linux shell
`sh stream_receive.sh`

Windows gitbash shell
`sh stream_receive_win.sh`

## Useful systemctl commands

Enable systemd service at startup
`systemctl enable stream`

Disable service
`systemctl disable stream`

Start service
`systemctl start stream`

Stop service
`systemctl stop stream`

Check status
`systemctl status stream`


