# RealSense Data Collection Instructions

## Hardware Requirements

- Intel RealSense D435i or D455 camera
- USB 3.0 connection (blue USB port recommended)
- Python 3.6+ with pyrealsense2, numpy, opencv-python, and Pillow

## Quick Start

### 1. Connect Camera

```bash
# Plug in your RealSense camera via USB 3.0
# Verify it's detected
lsusb | grep Intel
```

### 2. Run Data Collection

```bash
python3 process_realsense_callback.py --dst_dirpath ./my_recording_session
```

Replace `./my_recording_session` with your desired output directory path.

### 3. During Collection

- A window will display:
  - **Left side**: Live color image (640×480)
  - **Right side**: Depth visualization (640×480)
- Console prints status every ~1 second:
  ```
  Frame: 30 | IMU samples collected: 1200 | Queue size: 0
  Frame: 60 | IMU samples collected: 2400 | Queue size: 0
  ```
- **Queue size** should stay near 0 (indicates healthy data flow)

### 4. Stop Collection

Press **'q'** or **ESC** key in the display window

### 5. Review Summary

The script prints a summary on exit:
```
============================================================
Data collection complete!
============================================================
Total frames captured: 300
Total IMU samples collected: 4000
Expected IMU samples (200Hz): ~4000

Output files:
  - Frames index: /path/to/frames.csv
  - Accelerometer: /path/to/accelerometer.csv
  - Gyroscope: /path/to/gyroscope.csv
  ...
============================================================
```

## Output Directory Structure

After collection, your directory will contain:

```
my_recording_session/
├── accelerometer.csv           # Accelerometer data (200Hz)
├── gyroscope.csv              # Gyroscope data (200Hz)
├── frames.csv                 # Frame index with sync timestamps
├── image.txt                  # Legacy: list of all image paths
├── depth.txt                  # Legacy: list of all depth paths
├── intrinsics.txt             # Legacy: intrinsics file path
└── data/
    ├── intrinsics.npy         # Camera intrinsics matrix (3×3)
    ├── image/                 # RGB color images (640×480)
    │   ├── 2025-10-27T10:15:32.456.png
    │   ├── 2025-10-27T10:15:32.489.png
    │   └── ...
    ├── depth/                 # Depth as PNG (640×480, uint16 millimeters)
    │   ├── 2025-10-27T10:15:32.456.png
    │   └── ...
    └── depth_npy/             # Depth as NumPy (640×480, float32 meters)
        ├── 2025-10-27T10:15:32.456.npy
        └── ...
```

## Data Format Specifications

### 1. Accelerometer Data (`accelerometer.csv`)

**Sample Rate**: 200 Hz
**Format**: CSV with header

```csv
timestamp_ms,accel_x,accel_y,accel_z
12345678.123,0.0245,-9.8012,0.1234
12345683.123,0.0251,-9.7998,0.1240
12345688.123,0.0248,-9.8005,0.1237
```

| Column | Type | Unit | Description |
|--------|------|------|-------------|
| `timestamp_ms` | float | milliseconds | Device timestamp from RealSense |
| `accel_x` | float | m/s² | Acceleration along X-axis |
| `accel_y` | float | m/s² | Acceleration along Y-axis |
| `accel_z` | float | m/s² | Acceleration along Z-axis |

### 2. Gyroscope Data (`gyroscope.csv`)

**Sample Rate**: 200 Hz
**Format**: CSV with header

```csv
timestamp_ms,gyro_x,gyro_y,gyro_z
12345678.456,0.0012,0.0034,-0.0002
12345683.456,0.0015,0.0032,-0.0001
12345688.456,0.0013,0.0033,-0.0002
```

| Column | Type | Unit | Description |
|--------|------|------|-------------|
| `timestamp_ms` | float | milliseconds | Device timestamp from RealSense |
| `gyro_x` | float | rad/s | Angular velocity around X-axis |
| `gyro_y` | float | rad/s | Angular velocity around Y-axis |
| `gyro_z` | float | rad/s | Angular velocity around Z-axis |

### 3. Frames Index (`frames.csv`)

**Sample Rate**: 30 Hz (synchronized color + depth pairs)
**Format**: CSV with header

```csv
device_ts_ms,wallclock_iso,color_path,depth_png_path,depth_npy_path
176160346444,2025-10-27T10:15:32.456,/path/to/data/image/2025-10-27T10:15:32.456.png,/path/to/data/depth/2025-10-27T10:15:32.456.png,/path/to/data/depth_npy/2025-10-27T10:15:32.456.npy
176160346477,2025-10-27T10:15:32.489,/path/to/data/image/2025-10-27T10:15:32.489.png,/path/to/data/depth/2025-10-27T10:15:32.489.png,/path/to/data/depth_npy/2025-10-27T10:15:32.489.npy
```

| Column | Type | Description |
|--------|------|-------------|
| `device_ts_ms` | int | Device timestamp (milliseconds) for IMU synchronization |
| `wallclock_iso` | string | Wall-clock time in ISO 8601 format, used as filename |
| `color_path` | string | Absolute path to RGB image (PNG) |
| `depth_png_path` | string | Absolute path to depth PNG (millimeters) |
| `depth_npy_path` | string | Absolute path to depth NumPy (meters) |

**Usage**: This file links color/depth frames and enables synchronization with IMU data via `device_ts_ms`. Filenames use `wallclock_iso` for human readability.

### 4. Color Images (`data/image/*.png`)

**Resolution**: 640×480
**Format**: PNG (24-bit RGB)
**Naming**: ISO 8601 timestamp (e.g., `2025-10-27T10:15:32.456.png`)

### 5. Depth Images - PNG (`data/depth/*.png`)

**Resolution**: 640×480
**Format**: PNG (16-bit grayscale, mode 'I;16')
**Unit**: Millimeters (uint16)
**Naming**: ISO 8601 timestamp (e.g., `2025-10-27T10:15:32.456.png`)

**To load in Python**:
```python
from PIL import Image
import numpy as np

depth_mm = np.array(Image.open('2025-10-27T10:15:32.456.png'))  # uint16 millimeters
depth_m = depth_mm / 1000.0  # Convert to meters
```

### 6. Depth Images - NumPy (`data/depth_npy/*.npy`)

**Resolution**: 640×480
**Format**: NumPy binary (.npy)
**Type**: float32 or float64
**Unit**: Meters
**Naming**: ISO 8601 timestamp (e.g., `2025-10-27T10:15:32.456.npy`)

**To load in Python**:
```python
import numpy as np

depth_m = np.load('2025-10-27T10:15:32.456.npy')  # Already in meters (float)
```

### 7. Camera Intrinsics (`data/intrinsics.npy`)

**Format**: NumPy binary (.npy)
**Shape**: 3×3 matrix
**Type**: float64

```python
import numpy as np

K = np.load('data/intrinsics.npy')
# K = [[fx,  0, cx],
#      [ 0, fy, cy],
#      [ 0,  0,  1]]
```

| Parameter | Description |
|-----------|-------------|
| `fx` | Focal length in pixels (X-axis) |
| `fy` | Focal length in pixels (Y-axis) |
| `cx` | Principal point X-coordinate |
| `cy` | Principal point Y-coordinate |

## Expected Data Rates

For a **10-second recording** at default settings:

| Data Type | Rate | Expected Count |
|-----------|------|----------------|
| Color frames | 30 Hz | ~300 |
| Depth frames | 30 Hz | ~300 |
| Accelerometer samples | 200 Hz | ~2000 |
| Gyroscope samples | 200 Hz | ~2000 |
| **Total IMU samples** | 400 Hz | **~4000** |

## Data Verification Commands

After collection, verify data integrity:

```bash
# Count captured frames (subtract 1 for header)
wc -l my_recording_session/frames.csv

# Count IMU samples (subtract 1 for header)
wc -l my_recording_session/accelerometer.csv
wc -l my_recording_session/gyroscope.csv

# Check file sizes
du -sh my_recording_session/data/image/
du -sh my_recording_session/data/depth/
du -sh my_recording_session/data/depth_npy/

# List first few images
ls -lh my_recording_session/data/image/ | head -5
```

## Synchronization Strategy

### Dual Timestamp Approach
- **Filenames**: Use ISO 8601 wall-clock timestamps for human readability (e.g., `2025-10-27T10:15:32.456.png`)
- **Synchronization**: Use device timestamps from `frames.csv` for accurate sync with IMU data

All IMU and frame data share the same **device timestamp** time base (in milliseconds from RealSense hardware). This ensures accurate synchronization between:
- Color frames
- Depth frames
- Accelerometer readings
- Gyroscope readings

### How to Synchronize IMU with Frames

**Example**: Find all IMU samples between two consecutive frames

```python
import pandas as pd

# Load data
frames = pd.read_csv('frames.csv')
accel = pd.read_csv('accelerometer.csv')
gyro = pd.read_csv('gyroscope.csv')

# Get timestamps for frame 0 and frame 1
t0 = frames.loc[0, 'device_ts_ms']
t1 = frames.loc[1, 'device_ts_ms']

# Find all IMU samples in this interval
accel_between = accel[(accel['timestamp_ms'] >= t0) & (accel['timestamp_ms'] < t1)]
gyro_between = gyro[(gyro['timestamp_ms'] >= t0) & (gyro['timestamp_ms'] < t1)]

print(f"Between frames 0 and 1 ({t1-t0:.1f}ms):")
print(f"  Accelerometer samples: {len(accel_between)}")
print(f"  Gyroscope samples: {len(gyro_between)}")
```

Expected output: ~6-7 samples per sensor (200 Hz × 0.033s frame interval)

## Troubleshooting

### Issue: "No device connected"
- Ensure camera is plugged into USB 3.0 port (blue port)
- Try different USB port
- Check with `lsusb | grep Intel`

### Issue: Low IMU sample rate
- Check "Queue size" in console output
- If queue size keeps growing, your system may be too slow
- Try closing other applications

### Issue: "RuntimeError: Already streaming"
- Ensure no other RealSense applications are running
- Unplug and replug the camera
- Restart terminal session

### Issue: High queue size (> 100)
- IMU callback is generating data faster than main loop can write
- This is usually temporary and resolves quickly
- If persistent, check disk write speed

## Application: Visual-Inertial SLAM

This data format is designed for visual-inertial SLAM systems like:
- ORB-SLAM3
- VINS-Mono/VINS-Fusion
- OpenVINS
- Kimera

The synchronized color, depth, and IMU data enables:
- 6-DOF pose estimation
- Dense 3D reconstruction
- Loop closure detection
- Real-time tracking

## Legacy Files (Backward Compatibility)

The following files are maintained for compatibility with older scripts:

- `image.txt`: Line-separated list of image paths
- `depth.txt`: Line-separated list of depth PNG paths
- `intrinsics.txt`: Single line containing intrinsics.npy path

**Recommendation**: Use `frames.csv` for new projects as it provides better timestamp synchronization.

## Storage Requirements

Approximate storage per minute of recording:

| Data Type | Size/min | Notes |
|-----------|----------|-------|
| Color PNG | ~50 MB | 30 FPS, 640×480, PNG compression |
| Depth PNG | ~30 MB | 30 FPS, 640×480, 16-bit PNG |
| Depth NPY | ~35 MB | 30 FPS, 640×480, float32 uncompressed |
| IMU CSV | ~1 MB | 400 Hz total (accel + gyro) |
| **Total** | **~115 MB/min** | All data types |

Plan storage accordingly for long recording sessions.
