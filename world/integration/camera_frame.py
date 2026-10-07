"""Decode Gazebo camera frames without assuming packed, native-size RGB rows.

The real IMX296 acquisition adapter will have its own transport. Both adapters
must give the detector a BGR image plus an acquisition/receipt timestamp; a
Gazebo header stamp is not a host-monotonic acquisition timestamp.
"""

import cv2
import numpy as np


# Gazebo msgs PixelFormatType enum values used by the forward camera.
RGB_INT8 = 3
BGR_INT8 = 8


def gazebo_source_stamp_ns(msg):
    """Return the Gazebo simulation-clock stamp, never a host-monotonic age."""
    try:
        header = msg.header
        if not header.HasField("stamp"):
            return None
        sec, nsec = int(header.stamp.sec), int(header.stamp.nsec)
        if sec < 0 or not 0 <= nsec < 1_000_000_000:
            return None
        return sec * 1_000_000_000 + nsec
    except (AttributeError, TypeError, ValueError):
        return None


def decode_gazebo_bgr(msg, processing_width=None):
    """Return a contiguous BGR image; reject unknown or malformed layouts."""
    width = int(msg.width)
    height = int(msg.height)
    if width <= 0 or height <= 0:
        raise ValueError("camera dimensions must be positive")

    pixel_format = int(msg.pixel_format_type)
    if pixel_format not in (RGB_INT8, BGR_INT8):
        raise ValueError(f"unsupported camera pixel format {pixel_format}")

    packed_step = width * 3
    step = int(msg.step) or packed_step
    if step < packed_step or len(msg.data) != step * height:
        raise ValueError("camera row stride or data length is inconsistent")

    if processing_width is not None:
        processing_width = int(processing_width)
        if processing_width <= 0:
            raise ValueError("processing width must be positive")

    rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, step)
    image = rows[:, :packed_step].reshape(height, width, 3)
    if processing_width is not None and processing_width < width:
        target_height = max(1, round(height * processing_width / width))
        image = cv2.resize(image, (processing_width, target_height),
                           interpolation=cv2.INTER_AREA)
    if pixel_format == RGB_INT8:
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    else:
        image = np.ascontiguousarray(image)
    return image
