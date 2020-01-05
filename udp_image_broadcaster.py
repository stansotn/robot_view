# File: image_broadcaster.py by Stanislav Sotnikov.
# This script broadcasts image stream over UDP (Datagrams).

# "THE BEER-WARE LICENSE":
# stanislav.sotnikov145@gmail.com wrote this file.
# As long as you retain this notice you can do whatever you want with this code.
# If we meet some day, and you think this code is worth it, you can buy me a beer in return.

# Python standard libraries
import io
import math
import socket

# Libraries reqire install
import picamera

# Set the client's ip and desired port here!
PORT = 555
RECEIVER_IP = "192.168.1.74"

WIDTH = 1024
HEIGHT = 768

# This constant must hold the same value on receiver and transmitter side.
# Specifies the max amount of bytes sent per image datagram.
PACKET_MAX_SIZE = 8192

with picamera.PiCamera() as camera:

    broadcaster = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP

    image_stream = io.BytesIO()

    # Setup picamera.
    camera.resolution = (WIDTH, HEIGHT)
    camera.framerate = 10

    while True:
        # Capture an image.
        camera.capture(output=image_stream, format='jpeg', use_video_port=True, quality=10)

        # Create a memory view (pointer to the buffer).
        with image_stream.getbuffer() as image_stream_view:

            # Figure out how many packets are needed to send the whole image
            num_of_packets = math.ceil(len(image_stream_view)/PACKET_MAX_SIZE)
            #print("Num of packets to send: {n} packets".format(n = num_of_packets))

            #print("sending frame...")

            # Send a packet specifying frame lenght
            assert num_of_packets < 255, "Frame is too long."

            start_of_frame = b"SOF" + bytes([num_of_packets])
            debug = broadcaster.sendto(start_of_frame, (RECEIVER_IP, PORT))

            for i in range(num_of_packets):
    
                if i == num_of_packets - 1:
                    broadcaster.sendto(image_stream_view[i*PACKET_MAX_SIZE:], (RECEIVER_IP, PORT))

                else:
                    broadcaster.sendto(image_stream_view[i*PACKET_MAX_SIZE:(i+1)*PACKET_MAX_SIZE], (RECEIVER_IP, PORT))

        # Reset the buffer
        image_stream.seek(0)
        image_stream.truncate(0)
