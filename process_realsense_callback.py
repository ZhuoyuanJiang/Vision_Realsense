## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#####################################################
##     Align Depth to Color with Callback IMU      ##
#####################################################

# First import the library
import pyrealsense2 as rs
import numpy as np
import cv2
import os
import argparse
import csv
import threading
import time

from PIL import Image
from datetime import datetime

# Global variables for callback-based IMU collection
imu_csv_writer = None
imu_csv_file = None
imu_lock = threading.Lock()
imu_sample_count = 0

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

def imu_callback(frame):
    """
    Callback function for IMU data
    Called automatically when new IMU data arrives (200Hz)
    """
    global imu_csv_writer, imu_sample_count, imu_lock

    if imu_csv_writer is None:
        return

    try:
        with imu_lock:
            # Get timestamp
            timestamp = frame.get_timestamp()

            # Check if it's accelerometer or gyroscope
            motion_frame = frame.as_motion_frame()

            # We need to track both accel and gyro
            # This is a simplified approach - in production you'd want to match pairs
            if frame.get_profile().stream_type() == rs.stream.accel:
                # Store accelerometer data temporarily
                # In a real implementation, you'd want to pair this with the next gyro reading
                pass

            imu_sample_count += 1

    except Exception as e:
        print(f"IMU callback error: {e}")

def process_realsense_with_callback(dst_dirpath):
    global imu_csv_writer, imu_csv_file

    # Create a pipeline
    pipeline = rs.pipeline()

    # Create a config and configure the pipeline to stream
    config = rs.config()

    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
    config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 200)
    config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 200)

    # Create the directories
    if not os.path.exists(dst_dirpath):
        os.makedirs(dst_dirpath)

    data_dirpath = os.path.join(dst_dirpath, 'data')
    image_dirpath = os.path.join(data_dirpath, 'image')
    depth_dirpath = os.path.join(data_dirpath, 'depth')
    depth_npy_dirpath = os.path.join(data_dirpath, 'depth_npy')

    for dirpath in [data_dirpath, image_dirpath, depth_dirpath, depth_npy_dirpath]:
        if not os.path.exists(dirpath):
            os.makedirs(dirpath)

    # Open files
    image_txt_file = open(os.path.join(dst_dirpath, 'image.txt'), 'w')
    depth_txt_file = open(os.path.join(dst_dirpath, 'depth.txt'), 'w')
    intrinsics_txt_file = open(os.path.join(dst_dirpath, 'intrinsics.txt'), 'w')

    # Open CSV file for continuous IMU data
    imu_csv_file = open(os.path.join(dst_dirpath, 'imu_continuous.csv'), 'w', newline='')
    imu_csv_writer = csv.writer(imu_csv_file)
    imu_csv_writer.writerow(['timestamp_ms', 'accel_x', 'accel_y', 'accel_z', 'gyro_x', 'gyro_y', 'gyro_z'])

    # Start pipeline with callback
    queue = rs.frame_queue(capacity=1000)  # Large queue for IMU data

    profile = pipeline.start(config, queue)

    # Getting the depth sensor's depth scale
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    print('Depth Scale: {}'.format(depth_scale))

    # Get camera intrinsics
    intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()

    intrinsics_matrix = np.array([
        [intrinsics.fx, 0, intrinsics.ppx],
        [0, intrinsics.fy, intrinsics.ppy],
        [0, 0, 1.0]
    ])
    print('Camera intrinsics matrix:')
    print(intrinsics_matrix)

    intrinsics_path = os.path.join(data_dirpath, 'intrinsics.npy')
    np.save(intrinsics_path, intrinsics_matrix)

    # Create align object
    align = rs.align(rs.stream.color)

    stream_idx = 0
    last_accel = None
    last_gyro = None

    # Streaming loop
    try:
        print("Starting data collection with callback-based IMU...")
        print("Press 'q' to stop")
        print(f"IMU data being saved to: {os.path.join(dst_dirpath, 'imu_continuous.csv')}")

        while True:
            # Process all available frames in queue
            frames_available = queue.poll_for_frames()

            if frames_available:
                frames = frames_available.as_frameset()

                # Check for IMU frames first
                accel_frame = frames.first_or_default(rs.stream.accel)
                gyro_frame = frames.first_or_default(rs.stream.gyro)

                # Collect IMU data at full rate
                if accel_frame:
                    last_accel = accel_frame.as_motion_frame().get_motion_data()

                if gyro_frame:
                    last_gyro = gyro_frame.as_motion_frame().get_motion_data()

                # Write IMU data when we have both
                if last_accel and last_gyro:
                    timestamp = frames.get_timestamp()
                    with imu_lock:
                        imu_csv_writer.writerow([
                            timestamp,
                            last_accel.x,
                            last_accel.y,
                            last_accel.z,
                            last_gyro.x,
                            last_gyro.y,
                            last_gyro.z
                        ])
                    # Reset for next pair
                    last_accel = None
                    last_gyro = None

                # Process image/depth frames
                color_frame = frames.get_color_frame()
                depth_frame = frames.get_depth_frame()

                if color_frame and depth_frame:
                    timestamp = datetime.now().isoformat(timespec='milliseconds')
                    filename = '{}.png'.format(timestamp)

                    # Align depth to color
                    aligned_frames = align.process(frames)
                    aligned_depth_frame = aligned_frames.get_depth_frame()
                    color_frame = aligned_frames.get_color_frame()

                    if aligned_depth_frame and color_frame:
                        depth_image = np.asanyarray(aligned_depth_frame.get_data()) * depth_scale
                        color_image = np.asanyarray(color_frame.get_data())

                        # Save images
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

                        # Render for display
                        color_image_bgr = cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)
                        depth_colormap = np.expand_dims(depth_image, -1)
                        depth_colormap = 255.0 * depth_colormap / np.max(depth_colormap)
                        depth_colormap = np.tile(depth_colormap, (1, 1, 3)).astype(np.uint8)

                        images = np.hstack((color_image_bgr, depth_colormap))

                        # Display status
                        if stream_idx % 30 == 0:
                            print(f"Frame: {stream_idx}, IMU samples collected: {imu_sample_count}")

                        cv2.namedWindow('Stream', cv2.WINDOW_NORMAL)
                        cv2.imshow('Stream', images)

                        stream_idx += 1

            # Flush IMU data periodically
            if stream_idx % 10 == 0:
                imu_csv_file.flush()

            key = cv2.waitKey(1)
            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
                break

    finally:
        # Clean up
        pipeline.stop()

        # Close all files
        image_txt_file.close()
        depth_txt_file.close()
        intrinsics_txt_file.close()
        imu_csv_file.close()

        print(f"\nData collection complete!")
        print(f"Total IMU samples collected: {imu_sample_count}")
        print(f"Continuous IMU data saved to: {os.path.join(dst_dirpath, 'imu_continuous.csv')}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--dst_dirpath', type=str, required=True,
                       help='Path to destination directory')
    args = parser.parse_args()

    process_realsense_with_callback(args.dst_dirpath)