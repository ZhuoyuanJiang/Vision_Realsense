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
imu_sample_count = 0

def save_depth_png(depth_m, path):
    '''
    Saves a depth map as 16-bit PNG in millimeters
    Args:
        depth_m : numpy[float32/64]
            depth map in meters
        path : str
            path to store depth PNG
    '''
    depth_mm_u16 = np.clip(depth_m * 1000.0, 0, 65535).astype(np.uint16)
    Image.fromarray(depth_mm_u16, mode='I;16').save(path)

def save_raw_depth(z, path):
    '''
    saves raw depth as npy
    '''
    np.save(path, z)

def motion_callback(frame):
    """
    Unified callback for motion data (both accel and gyro)
    Called automatically by RealSense SDK when new IMU data arrives (200Hz)
    """
    global imu_sample_count

    try:
        stream_type = frame.get_profile().stream_type()
        motion_data = frame.as_motion_frame().get_motion_data()
        timestamp = frame.get_timestamp()  # Device timestamp in milliseconds

        item = {
            'timestamp': timestamp,
            'x': motion_data.x,
            'y': motion_data.y,
            'z': motion_data.z
        }

        if stream_type == rs.stream.accel:
            item['type'] = 'accel'
        elif stream_type == rs.stream.gyro:
            item['type'] = 'gyro'
        else:
            return

        imu_data_queue.put(item)  # queue.Queue is thread-safe, no lock needed
        imu_sample_count += 1

    except Exception as e:
        print(f"Motion callback error: {e}")

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

    # Create a pipeline
    pipeline = rs.pipeline()

    # Create a config and configure the pipeline to stream
    config = rs.config()

    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
    # IMU streams are managed separately via motion_sensor.start(callback)

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

    # Open frames index CSV to tie color/depth with timestamps
    frames_csv_file = open(os.path.join(dst_dirpath, 'frames.csv'), 'w', newline='')
    frames_csv_writer = csv.writer(frames_csv_file)
    frames_csv_writer.writerow(['device_ts_ms', 'wallclock_iso', 'color_path', 'depth_png_path', 'depth_npy_path'])

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

    # Write intrinsics path once (not every frame)
    intrinsics_txt_file.write(intrinsics_path + '\n')

    # Create align object
    align = rs.align(rs.stream.color)

    # Register callback for motion sensor (single sensor for both accel and gyro)
    device = profile.get_device()

    motion_sensor = None
    accel_profile = None
    gyro_profile = None

    for sensor in device.query_sensors():
        if sensor.is_motion_sensor():
            motion_sensor = sensor
            for profile_item in sensor.get_stream_profiles():
                stream_type = profile_item.stream_type()
                if stream_type == rs.stream.accel and accel_profile is None:
                    accel_profile = profile_item
                elif stream_type == rs.stream.gyro and gyro_profile is None:
                    gyro_profile = profile_item

    if motion_sensor is None or accel_profile is None or gyro_profile is None:
        print("ERROR: Motion sensor or profiles not found!")
        pipeline.stop()
        return

    # Open both profiles together on the SAME sensor
    motion_sensor.open([accel_profile, gyro_profile])
    motion_sensor.start(motion_callback)

    stream_idx = 0

    # Create OpenCV window once (outside loop)
    cv2.namedWindow('Stream', cv2.WINDOW_NORMAL)

    # Streaming loop
    try:
        print("Starting data collection with callback-based IMU...")
        print("Press 'q' to stop")
        print(f"Accelerometer data: {os.path.join(dst_dirpath, 'accelerometer.csv')}")
        print(f"Gyroscope data: {os.path.join(dst_dirpath, 'gyroscope.csv')}\n")

        while True:
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

            # Get timestamps - device timestamp for IMU sync, ISO for filenames
            device_ts = int(color_frame.get_timestamp())  # milliseconds
            wallclock_ts = datetime.now().isoformat(timespec='milliseconds')

            # Generate filenames from ISO timestamp (like original)
            png_filename = f"{wallclock_ts}.png"
            npy_filename = f"{wallclock_ts}.npy"

            # Build paths
            image_path = os.path.join(image_dirpath, png_filename)
            depth_png_path = os.path.join(depth_dirpath, png_filename)
            depth_npy_path = os.path.join(depth_npy_dirpath, npy_filename)

            # Process depth: raw uint16 -> meters (float)
            raw_z16 = np.asanyarray(aligned_depth_frame.get_data())
            depth_m = raw_z16 * depth_scale

            # Save color image
            color_image = np.asanyarray(color_frame.get_data())
            Image.fromarray(color_image).save(image_path)

            # Save depth as PNG (millimeters, uint16) and NPY (meters, float)
            save_depth_png(depth_m, depth_png_path)
            save_raw_depth(depth_m, depth_npy_path)

            # Write to legacy text files
            image_txt_file.write(image_path + '\n')
            depth_txt_file.write(depth_png_path + '\n')

            # Write to frames index CSV
            frames_csv_writer.writerow([device_ts, wallclock_ts, image_path, depth_png_path, depth_npy_path])

            # Render images for display
            color_image_bgr = cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)

            # Visualize depth with divide-by-zero protection
            viz_depth = depth_m.copy()
            max_depth = np.max(viz_depth)
            if max_depth > 0:
                viz_depth = (viz_depth / max_depth) * 255.0
            viz_depth = np.tile(viz_depth[..., None], (1, 1, 3)).astype(np.uint8)

            images = np.hstack((color_image_bgr, viz_depth))

            # Display status and periodic flush
            if stream_idx % 30 == 0:
                queue_size = imu_data_queue.qsize()
                print(f"Frame: {stream_idx} | IMU samples collected: {imu_sample_count} | Queue size: {queue_size}")
                frames_csv_file.flush()  # Flush frames index periodically

            cv2.imshow('Stream', images)
            key = cv2.waitKey(1)

            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
                break

            stream_idx += 1

    finally:
        # Stop motion sensor FIRST to prevent race condition
        if motion_sensor:
            motion_sensor.stop()
            motion_sensor.close()

        # NOW safely drain remaining IMU data from queue
        print("\nProcessing remaining IMU data...")
        process_imu_queue_to_csv(accel_csv_writer, gyro_csv_writer)

        # Stop pipeline
        pipeline.stop()

        # Close all files
        image_txt_file.close()
        depth_txt_file.close()
        intrinsics_txt_file.close()
        accel_csv_file.close()
        gyro_csv_file.close()
        frames_csv_file.close()

        print(f"\n{'='*60}")
        print(f"Data collection complete!")
        print(f"{'='*60}")
        print(f"Total frames captured: {stream_idx}")
        print(f"Total IMU samples collected: {imu_sample_count}")
        print(f"Expected IMU samples (200Hz): ~{stream_idx * 200 // 30}")
        print(f"\nOutput files:")
        print(f"  - Frames index: {os.path.join(dst_dirpath, 'frames.csv')}")
        print(f"  - Accelerometer: {os.path.join(dst_dirpath, 'accelerometer.csv')}")
        print(f"  - Gyroscope: {os.path.join(dst_dirpath, 'gyroscope.csv')}")
        print(f"  - Images: {image_dirpath}")
        print(f"  - Depth PNG (mm): {depth_dirpath}")
        print(f"  - Depth NPY (m): {depth_npy_dirpath}")
        print(f"  - Intrinsics: {intrinsics_path}")
        print(f"{'='*60}\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--dst_dirpath', type=str, required=True,
                       help='Path to destination directory')
    args = parser.parse_args()

    process_realsense_with_callback(args.dst_dirpath)