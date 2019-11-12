# File: udp_image_listener.py by Stanislav Sotnikov.
# This script listenes to a UDP image stream, decodes and shows the images.

# "THE BEER-WARE LICENSE": 
# stanislav.sotnikov145@gmail.com wrote this file. 
# As long as you retain this notice you can do whatever you want with this code. 
# If we meet some day, and you think this code is worth it, you can buy me a beer in return.

# Python standard libraries
import io
import time
import socket
import threading
import collections

# Libraries reqire install
import numpy as np
import cv2

PORT = 555
PACKET_MAX_SIZE = 8192

def reset_buffer(buffer):
    buffer.seek(0)
    buffer.truncate(0)

def do_networking(udp_socket, buffer):

    received_frame = io.BytesIO()

    # Packet Counter ensures that all frames were received intact
    packet_counter = -1

    while True:
        received_frame.write(udp_socket.recv(PACKET_MAX_SIZE))

        if received_frame.getvalue()[-4:-1] == b"SOF":

            if(packet_counter == 0):
                packet_counter = received_frame.getvalue()[-1]
                received_frame.truncate(len(received_frame.getbuffer()) - 4)
                buffer.append(received_frame.getvalue())

            else:
                packet_counter = received_frame.getvalue()[-1]

            reset_buffer(received_frame)

        else:
            packet_counter -= 1
            
            if(packet_counter < 0):
                packet_counter = -1
                reset_buffer(received_frame)

if __name__ == "__main__":

    # Serves as a buffer for receiving images to be passed to the networking thread
    image_queue = collections.deque(maxlen=5)

    # Setup UDP receiver.
    listener = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP
    listener.bind(('', PORT))

    # Launch networking thread.
    receive_thread = threading.Thread(target=do_networking, args=(listener, image_queue), daemon=True)
    receive_thread.start()

    frames = 0
    fps = 0
    t0 = time.time()

    while True:

        if len(image_queue) != 0:

            # Show Image
            image_bytes = np.frombuffer(image_queue.popleft(), dtype=np.uint8)

            image = cv2.imdecode(image_bytes, cv2.IMREAD_COLOR)

            if image is None:
                print("Frame Lost")
            
            else:
                frames += 1

                cv2.imshow("Hello World.png", image)
                cv2.waitKey(delay=1) # 1ms

        if(time.time() - t0 > 1):
            fps = round(frames/(time.time() - t0))
            print("fps ", fps)
            t0 = time.time()
            frames = 0

