# File: t265_broadcaster.py by Stanislav Sotnikov @ CCNY Robotics Lab.
# This script works with Intel RealSense T265 camera
# streaming out greyscale undistorted fisheye view from the left camera
# with the current pose coordinates embedded into the view image.

# Credit: This script was written following official intel's examples
# https://github.com/IntelRealSense/librealsense/blob/master/doc/t265.md
# Special attention and thanks to t265_stereo.py example

# "THE BEER-WARE LICENSE":
# stanislav.sotnikov145@gmail.com wrote this file.
# As long as you retain this notice you can do whatever you want with this code.
# If we meet some day, and you think this code is worth it, you can buy me a beer in return.

# Python standard libraries
import io
import socket
import math

# Libraries reqire install
# First import the library
import cv2
import numpy as np
import pyrealsense2 as rs

# Set the client's ip and desired port here!
PORT = 555
RECEIVER_IP = "192.168.1.10"

# This constant must hold the same value on receiver and transmitter side.
# Specifies the max amount of bytes sent per image datagram.
PACKET_MAX_SIZE = 8192

"""
Returns R, T transform from src to dst
"""
def get_extrinsics(src, dst):
    extrinsics = src.get_extrinsics_to(dst)
    R = np.reshape(extrinsics.rotation, [3,3]).T
    T = np.array(extrinsics.translation)
    return (R, T)

"""
Returns a camera matrix K from librealsense intrinsics
"""
def camera_matrix(intrinsics):
    return np.array([[intrinsics.fx,             0, intrinsics.ppx],
                     [            0, intrinsics.fy, intrinsics.ppy],
                     [            0,             0,              1]])

"""
Returns the fisheye distortion from librealsense intrinsics
"""
def fisheye_distortion(intrinsics):
    return np.array(intrinsics.coeffs[:4])

def quaternion_to_euler(x, y, z, w):

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    X = math.degrees(math.atan2(t0, t1))

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    Y = math.degrees(math.asin(t2))

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    Z = math.degrees(math.atan2(t3, t4))

    return X, Y, Z

frame_data = {"left"            : None,
              "right"           : None,
              "timestamp_ms"    : None,
              "translation_x"   : None,
              "translation_y"   : None,
              "translation_z"   : None,
              "Qw"              : None,
              "Qx"              : None,
              "Qy"              : None,
              "Qz"              : None
              }

if __name__ == "__main__":

    # Setup network interface.
    broadcaster = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP

    # Declare RealSense pipeline, encapsulating the actual device and sensors
    pipe = rs.pipeline()

    # Build config object and stream everything
    cfg = rs.config()
    cfg.enable_stream(rs.stream.pose)
    cfg.enable_stream(rs.stream.fisheye, 1)
    cfg.enable_stream(rs.stream.fisheye, 2)  

    # Start streaming with our callback
    #pipe.start(cfg, callback)
    pipe.start(cfg)

    min_disp = 0
    # must be divisible by 16
    num_disp = 112 - min_disp
    max_disp = min_disp + num_disp

    # Retreive the stream and intrinsic properties for both cameras
    profiles = pipe.get_active_profile()

    streams = {"left"  : profiles.get_stream(rs.stream.fisheye, 1).as_video_stream_profile()}

    intrinsics = {"left"  : streams["left"].get_intrinsics()}

    # Print information about both cameras
    print("Left camera:", intrinsics["left"])
    #print("Right camera:", intrinsics["right"])

    # Translate the intrinsics from librealsense into OpenCV
    K_left = camera_matrix(intrinsics["left"])
    D_left = fisheye_distortion(intrinsics["left"])

    (width, height) = (intrinsics["left"].width, intrinsics["left"].height)

    # We need to determine what focal length our undistorted images should have
    # in order to set up the camera matrices for initUndistortRectifyMap.  We
    # could use stereoRectify, but here we show how to derive these projection
    # matrices from the calibration and a desired height and field of view

    # We calculate the undistorted focal length:
    #
    #         h
    # -----------------
    #  \      |      /
    #    \    | f  /
    #     \   |   /
    #      \ fov /
    #        \|/
    stereo_fov_rad = 90 * (math.pi/180)  # 90 degree desired fov
    stereo_height_px = 300          # 300x300 pixel stereo output
    stereo_focal_px = stereo_height_px/2 / math.tan(stereo_fov_rad/2)

    # We set the left rotation to identity and the right rotation
    # the rotation between the cameras
    R_left = np.eye(3)

    # The stereo algorithm needs max_disp extra pixels in order to produce valid
    # disparity on the desired output region. This changes the width, but the
    # center of projection should be on the center of the cropped image
    stereo_width_px = stereo_height_px + max_disp
    stereo_size = (stereo_width_px, stereo_height_px)
    stereo_cx = (stereo_height_px - 1)/2 + max_disp
    stereo_cy = (stereo_height_px - 1)/2

    # Construct the left and right projection matrices, the only difference is
    # that the right projection matrix should have a shift along the x axis of
    # baseline*focal_length
    P_left = np.array([[stereo_focal_px, 0, stereo_cx, 0],
                       [0, stereo_focal_px, stereo_cy, 0],
                       [0, 0, 1, 0]])


    # Create an undistortion map for the left and right camera which applies the
    # rectification and undoes the camera distortion. This only has to be done
    # once
    m1type = cv2.CV_32FC1
    (lm1, lm2) = cv2.fisheye.initUndistortRectifyMap(K_left, 
                                                     D_left, 
                                                     R_left, 
                                                     P_left, 
                                                     stereo_size, 
                                                     m1type)

    undistort_rectify = (lm1, lm2)

    while True:
        # Wait for the next frameset.
        frameset = pipe.wait_for_frames()

        # Aquire fisheye and pose frames.
        f1 = frameset.get_fisheye_frame(1).as_video_frame()
        pose = frameset.get_pose_frame()

        if pose and f1:
         
            # Read data from the frameset.
            frame_data["translation_x"] = pose.get_pose_data().translation.x
            frame_data["translation_y"] = pose.get_pose_data().translation.y
            frame_data["translation_z"] = pose.get_pose_data().translation.z
            frame_data["Qw"] = pose.get_pose_data().rotation.w
            frame_data["Qx"] = pose.get_pose_data().rotation.x
            frame_data["Qy"] = pose.get_pose_data().rotation.y
            frame_data["Qz"] = pose.get_pose_data().rotation.z

            frame_data["left"] = np.asanyarray(f1.get_data())
            frame_data["timestamp_ms"] = frameset.get_timestamp()
            
            pitch, roll, yaw = quaternion_to_euler(-frame_data["Qz"], frame_data["Qx"], frame_data["Qy"], frame_data["Qw"])
            #pitch, yaw, roll = quaternion_to_euler(frame_data["Qx"], frame_data["Qy"], frame_data["Qz"], frame_data["Qw"])
            # Undistort and crop the center of the frames
            center_undistorted = cv2.remap(src=frame_data["left"],
                                           map1=undistort_rectify[0],
                                           map2=undistort_rectify[1],
                                           interpolation=cv2.INTER_LINEAR)

            # Add coordinates to the camera view
            coordinate_text = "X:{x}, Y:{y}, Z:{z}, O:{o}".format(x=round(frame_data["translation_x"], 3), 
                                                           y=round(frame_data["translation_y"], 3),
                                                           z=round(frame_data["translation_z"], 3),
                                                           o=round(yaw, 3))
            
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(center_undistorted, coordinate_text, (0, 295), font, 0.7, (255, 255, 255), 2, cv2.LINE_AA)

            # Encode view to send over udp.
            is_success, image_buffer = cv2.imencode(".jpg", center_undistorted)

            image_stream = io.BytesIO(image_buffer)

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
