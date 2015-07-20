ffserver -f /etc/ffserver.conf & ffmpeg -v verbose -r 30 -s 640x480 -f video4linux2 -i /dev/video0 http://localhost:8190/webcam.ffm
