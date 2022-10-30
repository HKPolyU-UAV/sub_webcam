gst-launch-1.0 -vc udpsrc port=5600 close-socket=false multicast-iface=false auto-multicast=true \
! application/x-rtp, payload=96 ! rtpjitterbuffer ! rtph264depay ! avdec_h264 \
! fpsdisplaysink  sync=false async=false --verbose