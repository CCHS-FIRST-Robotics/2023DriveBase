gst-launch-1.0 -vvv udpsrc port=5000 ! application/x-rtp,encoding-name=H265,payload=96 ! rtph265depay ! h265parse ! queue ! avdec_h265 ! autovideosink sync=false -e
