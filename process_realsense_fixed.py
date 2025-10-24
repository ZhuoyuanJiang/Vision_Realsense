## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#####################################################
##              Align Depth to Color               ##
#####################################################

# First import the library
import pyrealsense2 as rs
import numpy as np
import cv2
import os
import argparse
import csv
import threading
import queue
import time

from PIL import Image
from datetime import datetime

# Create a pipeline
pipeline = rs.pipeline()

# Create a config and configure the pipeline to stream
#  different resolutions of color and depth streams
config = rs.config()

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 200)  # 200Hz for IMU
config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 200)   # 200Hz for IMU

# Global queue for IMU data collection
imu_queue = queue.Queue()
stop_imu_thread = threading.Event()

def save_depth(z, path):
    '''
    Saves a depth map to a 16-bit PNG file
    Args:
        z : numpy[float32]
            depth map
        path : str
            path to store depth map
    '''
    z = np.uint32(z * 256.0)
    z = Image.fromarray(z, mode='I')
    z.save(path)

def save_raw_depth(z, path):
    '''
    saves raw depth as npy
    '''
    np.save(path, z)

def imu_collection_thread(pipeline, csv_writer):
    """
    Separate thread for continuous IMU data collection at high frequency
    Collects all IMU samples regardless of frame processing
    """
    try:
        while not stop_imu_thread.is_set():
            # Wait for IMU frames with short timeout
            frames = pipeline.wait_for_frames(timeout_ms=10)

            # Get timestamp in milliseconds
            timestamp = time.time() * 1000.0

            # Process accelerometer data
            accel_frame = frames.first_or_default(rs.stream.accel)
            gyro_frame = frames.first_or_default(rs.stream.gyro)

            if accel_frame and gyro_frame:
                # Get IMU timestamp from device
                device_timestamp = accel_frame.get_timestamp()

                accel_data = accel_frame.as_motion_frame().get_motion_data()
                gyro_data = gyro_frame.as_motion_frame().get_motion_data()

                # Write to CSV: timestamp, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z
                imu_row = [
                    device_timestamp,  # Use device timestamp for better accuracy
                    accel_data.x,
                    accel_data.y,
                    accel_data.z,
                    gyro_data.x,
                    gyro_data.y,
                    gyro_data.z
                ]

                # Add to queue for thread-safe writing
                imu_queue.put(imu_row)

    except Exception as e:
        print(f"IMU thread error: {e}")

def process_imu_queue(csv_writer):
    """
    Process queued IMU data and write to CSV
    """
    while not imu_queue.empty():
        try:
            imu_row = imu_queue.get_nowait()
            csv_writer.writerow(imu_row)
        except queue.Empty:
            break

def process_realsense(dst_dirpath):

    # Start streaming
    profile = pipeline.start(config)

    # Getting the depth sensor's depth scale (see rs-align example for explanation)
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    print('Depth Scale: {}'.format(depth_scale))

    # Get camera intrinsics
    intrinsics = profile \
        .get_stream(rs.stream.color) \
        .as_video_stream_profile() \
        .get_intrinsics()

    intrinsics_matrix = np.zeros((3, 3))
    intrinsics_matrix[0, 0] = intrinsics.fx
    intrinsics_matrix[1, 1] = intrinsics.fy
    intrinsics_matrix[0, 2] = intrinsics.ppx
    intrinsics_matrix[1, 2] = intrinsics.ppy
    intrinsics_matrix[2, 2] = 1.0
    print('Camera intrinsics matrix:')
    print(intrinsics_matrix)

    # Create an align object
    # rs.align allows us to perform alignment of depth frames to others frames
    # The "align_to" is the stream type to which we plan to align depth frames.
    align_to = rs.stream.color
    align = rs.align(align_to)

    # Create the directories
    if not os.path.exists(dst_dirpath):
        os.makedirs(dst_dirpath)

    data_dirpath = os.path.join(dst_dirpath, 'data')
    image_dirpath = os.path.join(data_dirpath, 'image')
    depth_dirpath = os.path.join(data_dirpath, 'depth')
    depth_npy_dirpath = os.path.join(data_dirpath, 'depth_npy')

    if not os.path.exists(data_dirpath):
        os.makedirs(data_dirpath)

    if not os.path.exists(image_dirpath):
        os.makedirs(image_dirpath)

    if not os.path.exists(depth_dirpath):
        os.makedirs(depth_dirpath)

    if not os.path.exists(depth_npy_dirpath):
        os.makedirs(depth_npy_dirpath)

    # Open the text files
    image_txt_file = open(os.path.join(dst_dirpath, 'image.txt'), 'w')
    depth_txt_file = open(os.path.join(dst_dirpath, 'depth.txt'), 'w')
    intrinsics_txt_file = open(os.path.join(dst_dirpath, 'intrinsics.txt'), 'w')

    # Open CSV file for continuous IMU data with proper headers
    imu_csv_file = open(os.path.join(dst_dirpath, 'imu_continuous.csv'), 'w', newline='')
    imu_csv_writer = csv.writer(imu_csv_file)

    # Write CSV header
    imu_csv_writer.writerow(['timestamp_ms', 'accel_x', 'accel_y', 'accel_z', 'gyro_x', 'gyro_y', 'gyro_z'])

    # Also create a frame-synced IMU file for reference
    frame_imu_file = open(os.path.join(dst_dirpath, 'frame_imu.csv'), 'w', newline='')
    frame_imu_writer = csv.writer(frame_imu_file)
    frame_imu_writer.writerow(['frame_timestamp', 'frame_idx', 'accel_x', 'accel_y', 'accel_z', 'gyro_x', 'gyro_y', 'gyro_z'])

    intrinsics_path = os.path.join(data_dirpath, 'intrinsics.npy')
    np.save(intrinsics_path, intrinsics_matrix)

    stream_idx = 0

    # Start IMU collection thread
    imu_thread = threading.Thread(target=imu_collection_thread, args=(pipeline, imu_csv_writer))
    imu_thread.daemon = True
    imu_thread.start()

    # Streaming loop
    try:
        print("Starting data collection...")
        print("Press 'q' to stop")
        print(f"IMU data being saved to: {os.path.join(dst_dirpath, 'imu_continuous.csv')}")

        while True:
            timestamp = datetime.now().isoformat(timespec='milliseconds')
            filename = '{}.png'.format(timestamp)

            # Get frameset of color and depth
            frames = pipeline.wait_for_frames()

            # Process IMU queue periodically
            process_imu_queue(imu_csv_writer)
            imu_csv_file.flush()  # Ensure data is written to disk

            # Align the depth frame to color frame
            aligned_frames = align.process(frames)

            # Get aligned frames
            aligned_depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()

            # Also get current IMU frame for frame-synced recording
            accel_frame = frames.first_or_default(rs.stream.accel)
            gyro_frame = frames.first_or_default(rs.stream.gyro)

            # Validate that both frames are valid
            if not aligned_depth_frame or not color_frame:
                continue

            # Save frame-synced IMU data
            if accel_frame and gyro_frame:
                accel_data = accel_frame.as_motion_frame().get_motion_data()
                gyro_data = gyro_frame.as_motion_frame().get_motion_data()

                frame_imu_writer.writerow([
                    timestamp,
                    stream_idx,
                    accel_data.x,
                    accel_data.y,
                    accel_data.z,
                    gyro_data.x,
                    gyro_data.y,
                    gyro_data.z
                ])
                frame_imu_file.flush()

            depth_image = np.asanyarray(aligned_depth_frame.get_data()) * depth_scale
            color_image = np.asanyarray(color_frame.get_data())

            # Save to disk
            image = Image.fromarray(color_image)
            image_path = os.path.join(image_dirpath, filename)
            image.save(image_path)
            image_txt_file.write(image_path + '\n')

            depth_path = os.path.join(depth_dirpath, filename)
            depth_npy_path = os.path.join(depth_npy_dirpath, filename)
            save_depth(depth_image, depth_path)
            save_raw_depth(depth_image, depth_npy_path)
            depth_txt_file.write(depth_path + '\n')

            intrinsics_txt_file.write(intrinsics_path + '\n')

            # Render images
            color_image = cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)
            depth_colormap = np.expand_dims(depth_image, -1)
            depth_colormap = 255.0 * depth_colormap / np.max(depth_colormap)
            depth_colormap = np.tile(depth_colormap, (1, 1, 3)).astype(np.uint8)

            images = np.hstack((color_image, depth_colormap))

            # Display IMU status
            if stream_idx % 30 == 0:  # Print every second
                queue_size = imu_queue.qsize()
                print(f"Frame: {stream_idx}, IMU queue size: {queue_size}")

            cv2.namedWindow('Stream', cv2.WINDOW_NORMAL)
            cv2.imshow('Stream', images)
            key = cv2.waitKey(1)

            # Press esc or 'q' to close the image window
            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
                break

            stream_idx += 1

    finally:
        # Stop IMU thread
        stop_imu_thread.set()
        if imu_thread.is_alive():
            imu_thread.join(timeout=1.0)

        # Process remaining IMU data
        process_imu_queue(imu_csv_writer)

        # Close all files
        image_txt_file.close()
        depth_txt_file.close()
        intrinsics_txt_file.close()
        imu_csv_file.close()
        frame_imu_file.close()

        pipeline.stop()
        print(f"\nData collection complete!")
        print(f"Continuous IMU data saved to: {os.path.join(dst_dirpath, 'imu_continuous.csv')}")
        print(f"Frame-synced IMU data saved to: {os.path.join(dst_dirpath, 'frame_imu.csv')}")


if __name__ == "__main__":

    parser = argparse.ArgumentParser()

    parser.add_argument('--dst_dirpath',
    type=str, required=True, help='Path to destination directory')

    args = parser.parse_args()

    # Process the rosbag
    process_realsense(args.dst_dirpath)