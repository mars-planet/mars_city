"""A BGE client which displays a streaming feed in a Blender Texture
 with soft real-time performance
 """
import bge

# Configuration variables for the streaming server
STREAM_SERVER_IP = "http://localhost"
STREAM_SERVER_PORT = "8290"
STREAM_FILE = "webcam.mjpeg"
FFMPEG_PARAM = STREAM_SERVER_IP + ":" +  STREAM_SERVER_PORT + "/" + STREAM_FILE

# get the controller the script is attached to
controller = bge.logic.getCurrentController()
obj = controller.owner
# get the reference pointer (ID) of the internal texture
ID = bge.texture.materialID(obj, 'IMmain_frame.png')

# Placeholder for debugging in the Blender terminal
print("====start")

# set up a video streaming service
# like ffmpeg/icecast/vlc on the server side and read here.
# ffmpeg is optimized for this.
# Blender natively supports ffmpeg

def main():
    print('refresh')
    if not hasattr(bge.logic, 'video'):
        # Get an instance of the video texture
        bge.logic.video = bge.texture.Texture(obj, ID)

        # a ffmpeg server is streaming the feed on the IP:PORT/FILE
        # specified in FFMPEG_PARAM,
        # BGE reads the stream from the mjpeg file.

        bge.logic.video.source = bge.texture.VideoFFmpeg(FFMPEG_PARAM)
        bge.logic.video.source.play()
    bge.logic.video.refresh(True)

# Placeholder for debugging in the Blender Terminal
print("====end")
