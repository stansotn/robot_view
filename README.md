# udp_image_stream
This project is a pair of python scripts that allow to stream images over UDP Datagrams.

## Project Structure
[udp_image_broadcaster.py](https://github.com/Slavon145/udp_image_stream/blob/master/udp_image_broadcaster.py) - Broadcasts a raspi camera image stream over udp.  
[udp_image_listener.py](https://github.com/Slavon145/udp_image_stream/blob/master/udp_image_listener.py) - Receives images and displays the stream on the client's monitor.  
[t265_broadcaster.py](https://github.com/Slavon145/udp_image_stream/blob/master/t265_broadcaster.py) - Broadcasts greyscale view with embedded into the image pose from Intel RealSense T265 Camera.  

## Why?
My raspberry pi powered robot was happily rolling around hallways of my uni and I was looking to add a video stream support to be able to drive around without having the robot in my physical eyesight. I ran into many tutorials and they all used tcp connection to stream the video, which means that if my robot finds itself in the room next door, the stream starts dropping frames introducing a 5~10s delay. The tcp approach might be fine for surveillance, but it was unacceptable for something that requires an instant response. I wanted the current image or nothing and udp was theoretically the answer!

## How?
I spent some time on [Adrian's tutorial](https://www.pyimagesearch.com/2019/04/15/live-video-streaming-over-network-with-opencv-and-imagezmq/) that uses [imagezmq](https://github.com/jeffbass/imagezmq#introduction) and and some other libraries that abstract sockets. I was spooked by the bare sockets and I was looking for a suitable library that just does the thing (please point me at one). Finally, I gave up and decided to experiment with bare socket attempting to reasonably minimize calls to third party packages.

## What?
I wrote this pair of python scripts implementing a lost packet prone udp image streaming using mostly bare sockets and standard python libraries. The resulting code turned out to be not much longer and even simpler than for instance the zmq implementation. The video stream is instant and the result of driving the robot to the room next door is just lower fps, no latency!
