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
import queue
import time

from PIL import Image
from datetime import datetime

# Global variables for callback-based IMU collection
imu_data_queue = queue.Queue()  # Thread-safe queue for IMU data
imu_lock = threading.Lock()
imu_sample_count = 0

# Separate CSV writers for accel and gyro (will be set in main function)
accel_csv_writer = None
gyro_csv_writer = None

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

def accel_callback(frame):
    """
    Callback function for accelerometer data
    Called automatically by RealSense SDK when new accel data arrives (200Hz)
    """
    global accel_csv_writer, imu_sample_count, imu_lock

    try:
        with imu_lock:
            motion_data = frame.as_motion_frame().get_motion_data()
            timestamp = frame.get_timestamp()  # Device timestamp in milliseconds

            # Store in queue with type marker
            imu_data_queue.put({
                'type': 'accel',
                'timestamp': timestamp,
                'x': motion_data.x,
                'y': motion_data.y,
                'z': motion_data.z
            })
            imu_sample_count += 1

    except Exception as e:
        print(f"Accel callback error: {e}")

def gyro_callback(frame):
    """
    Callback function for gyroscope data
    Called automatically by RealSense SDK when new gyro data arrives (200Hz)
    """
    global gyro_csv_writer, imu_sample_count, imu_lock

    try:
        with imu_lock:
            motion_data = frame.as_motion_frame().get_motion_data()
            timestamp = frame.get_timestamp()  # Device timestamp in milliseconds

            # Store in queue with type marker
            imu_data_queue.put({
                'type': 'gyro',
                'timestamp': timestamp,
                'x': motion_data.x,
                'y': motion_data.y,
                'z': motion_data.z
            })
            imu_sample_count += 1

    except Exception as e:
        print(f"Gyro callback error: {e}")

def process_imu_queue_to_csv(accel_writer, gyro_writer):
    """
    Process all queued IMU data and write to separate CSV files
    """
    items_written = 0
    while not imu_data_queue.empty():
        try:
            imu_data = imu_data_queue.get_nowait()

            if imu_data['type'] == 'accel':
                accel_writer.writerow([
                    imu_data['timestamp'],
                    imu_data['x'],
                    imu_data['y'],
                    imu_data['z']
                ])
            elif imu_data['type'] == 'gyro':
                gyro_writer.writerow([
                    imu_data['timestamp'],
                    imu_data['x'],
                    imu_data['y'],
                    imu_data['z']
                ])

            items_written += 1

        except queue.Empty:
            break

    return items_written

def process_realsense_with_callback(dst_dirpath):
    global accel_csv_writer, gyro_csv_writer

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

    # Open separate CSV files for accelerometer and gyroscope (200Hz each)
    accel_csv_file = open(os.path.join(dst_dirpath, 'accelerometer.csv'), 'w', newline='')
    gyro_csv_file = open(os.path.join(dst_dirpath, 'gyroscope.csv'), 'w', newline='')

    accel_csv_writer = csv.writer(accel_csv_file)
    gyro_csv_writer = csv.writer(gyro_csv_file)

    # Write CSV headers
    accel_csv_writer.writerow(['timestamp_ms', 'accel_x', 'accel_y', 'accel_z'])
    gyro_csv_writer.writerow(['timestamp_ms', 'gyro_x', 'gyro_y', 'gyro_z'])

    # Start pipeline
    profile = pipeline.start(config)

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

    # Register callbacks for IMU sensors
    device = profile.get_device()

    accel_sensor = None
    gyro_sensor = None

    for sensor in device.query_sensors():
        if sensor.is_motion_sensor():
            for profile_item in sensor.get_stream_profiles():
                if profile_item.stream_type() == rs.stream.accel:
                    accel_sensor = sensor
                elif profile_item.stream_type() == rs.stream.gyro:
                    gyro_sensor = sensor

    if accel_sensor is None or gyro_sensor is None:
        print("ERROR: Could not find IMU sensors!")
        pipeline.stop()
        return

    accel_sensor.open(accel_sensor.get_stream_profiles()[0])
    accel_sensor.start(accel_callback)

    gyro_sensor.open(gyro_sensor.get_stream_profiles()[0])
    gyro_sensor.start(gyro_callback)

    stream_idx = 0

    # Streaming loop
    try:
        print("Starting data collection with callback-based IMU...")
        print("Press 'q' to stop")
        print(f"Accelerometer data: {os.path.join(dst_dirpath, 'accelerometer.csv')}")
        print(f"Gyroscope data: {os.path.join(dst_dirpath, 'gyroscope.csv')}\n")

        while True:
            timestamp = datetime.now().isoformat(timespec='milliseconds')
            filename = '{}.png'.format(timestamp)

            # Get frameset of color and depth
            frames = pipeline.wait_for_frames()

            # Process and write queued IMU data
            items_written = process_imu_queue_to_csv(accel_csv_writer, gyro_csv_writer)

            # Flush CSV files periodically
            if items_written > 0:
                accel_csv_file.flush()
                gyro_csv_file.flush()

            # Align the depth frame to color frame
            aligned_frames = align.process(frames)

            # Get aligned frames
            aligned_depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()

            # Validate that both frames are valid
            if not aligned_depth_frame or not color_frame:
                continue

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
            color_image_bgr = cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)
            depth_colormap = np.expand_dims(depth_image, -1)
            depth_colormap = 255.0 * depth_colormap / np.max(depth_colormap)
            depth_colormap = np.tile(depth_colormap, (1, 1, 3)).astype(np.uint8)

            images = np.hstack((color_image_bgr, depth_colormap))

            # Display status
            if stream_idx % 30 == 0:
                queue_size = imu_data_queue.qsize()
                print(f"Frame: {stream_idx} | IMU samples collected: {imu_sample_count} | Queue size: {queue_size}")

            cv2.namedWindow('Stream', cv2.WINDOW_NORMAL)
            cv2.imshow('Stream', images)
            key = cv2.waitKey(1)

            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
                break

            stream_idx += 1

    finally:
        # Process remaining IMU data
        print("\nProcessing remaining IMU data...")
        process_imu_queue_to_csv(accel_csv_writer, gyro_csv_writer)

        # Stop IMU sensors
        if accel_sensor:
            accel_sensor.stop()
            accel_sensor.close()
        if gyro_sensor:
            gyro_sensor.stop()
            gyro_sensor.close()

        # Stop pipeline
        pipeline.stop()

        # Close all files
        image_txt_file.close()
        depth_txt_file.close()
        intrinsics_txt_file.close()
        accel_csv_file.close()
        gyro_csv_file.close()

        print(f"\n{'='*60}")
        print(f"Data collection complete!")
        print(f"{'='*60}")
        print(f"Total frames captured: {stream_idx}")
        print(f"Total IMU samples collected: {imu_sample_count}")
        print(f"Expected IMU samples (200Hz): ~{stream_idx * 200 // 30}")
        print(f"\nOutput files:")
        print(f"  - Accelerometer: {os.path.join(dst_dirpath, 'accelerometer.csv')}")
        print(f"  - Gyroscope: {os.path.join(dst_dirpath, 'gyroscope.csv')}")
        print(f"  - Images: {image_dirpath}")
        print(f"  - Depth: {depth_dirpath}")
        print(f"{'='*60}\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--dst_dirpath', type=str, required=True,
                       help='Path to destination directory')
    args = parser.parse_args()

    process_realsense_with_callback(args.dst_dirpath)