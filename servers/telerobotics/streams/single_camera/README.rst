======================
blender-remote-stream
======================
Server and client configuration which allows 
one to stream a video over the network
to a host running a Blender game engine instance. 

**Applications** include integrating live video inside a game, 
Immersive Virtual Reality simulations, real-time video support

Steps to get the setup working
================================

1. Configure a ffmpeg server on the remote machine.

2. For this, copy the ffserver.conf file
   in the /etc/ directory of the remote machine.
   
3. Start the server via the ffmpeg command specified in webcam.sh 

4. The server reads from /dev/video0 and serves it on the listening port.

5. The client machine has the Blender scene and script running.