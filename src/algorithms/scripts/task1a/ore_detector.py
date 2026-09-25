#!/usr/bin/env python3

"""
*****************************************************************************************
*
*                       StrataCobot (SC) Theme
*                          eYRC 2026-27
*
* Task 1A - Ore Detection and TF Publishing
*
*****************************************************************************************
"""

import os
import sys
import cv2
import rclpy
import tf2_ros
import numpy as np

from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped, PointStamped

from tf2_geometry_msgs import do_transform_point


# =============================================================================
# TASK CONSTANTS
# =============================================================================

ore_types = [
    'azurite_ore',
    'malachite_ore',
    'vanadinite_ore'
]

color_topic = '/camera/camera/color/image_raw'

depth_topic = (
    '/camera/camera/aligned_depth_to_color/image_raw'
)

camera_info_topic = (
    '/camera/camera/color/camera_info'
)

base_frame = 'base_link'


# The ore collision box in the official model is:
# 0.1016 x 0.1016 x 0.0762 m
#
# Camera measures the top face.
# Required TF represents the centre of the ore.
#
# Therefore:
# 0.0762 / 2 = 0.0381 m
ORE_HALF_HEIGHT = 0.0381


# =============================================================================
# IMAGE PROCESSING
# =============================================================================

def detect_ores(image):
    """
    Detect all coloured ore faces.

    Returns:
        center_ore_list : list of (cX, cY)
        ore_type_list   : list of ore types
        contour_list    : list of contours
    """

    center_ore_list = []
    ore_type_list = []
    contour_list = []

    # -------------------------------------------------------------------------
    # BGR -> HSV
    # -------------------------------------------------------------------------

    hsv = cv2.cvtColor(
        image,
        cv2.COLOR_BGR2HSV
    )

    # -------------------------------------------------------------------------
    # Conveyor ROI mask
    # -------------------------------------------------------------------------

    h_img, w_img = image.shape[:2]

    roi_mask = np.zeros(
        (h_img, w_img),
        dtype=np.uint8
    )

    # Generous region covering the conveyor area
    # Shifted x1 to 0.55 to exclude the robot arm's base
    roi_x1 = int(w_img * 0.55)
    roi_y1 = int(h_img * 0.35)
    roi_x2 = w_img
    roi_y2 = h_img

    roi_mask[roi_y1:roi_y2, roi_x1:roi_x2] = 255

    # -------------------------------------------------------------------------
    # HSV ranges
    # -------------------------------------------------------------------------

    color_ranges = {

        # BLUE -> AZURITE
        'azurite_ore': (
            np.array([90, 70, 40], dtype=np.uint8),
            np.array([135, 255, 255], dtype=np.uint8)
        ),

        # GREEN -> MALACHITE
        'malachite_ore': (
            np.array([35, 60, 40], dtype=np.uint8),
            np.array([90, 255, 255], dtype=np.uint8)
        ),

        # ORANGE -> VANADINITE
        # S lowered to 100 to catch both ores under different lighting
        'vanadinite_ore': (
            np.array([0, 100, 80], dtype=np.uint8),
            np.array([25, 255, 255], dtype=np.uint8)
        )
    }

    kernel = np.ones(
        (5, 5),
        np.uint8
    )

    # -------------------------------------------------------------------------
    # Detect each ore colour
    # -------------------------------------------------------------------------

    for ore_type in ore_types:

        lower, upper = color_ranges[ore_type]

        mask = cv2.inRange(
            hsv,
            lower,
            upper
        )

        mask = cv2.bitwise_and(
            mask,
            roi_mask
        )

        # Remove small noise
        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_OPEN,
            kernel
        )

        # Fill small gaps
        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_CLOSE,
            kernel
        )

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        valid = []

        # ---------------------------------------------------------------------
        # Filter contours
        # ---------------------------------------------------------------------

        for contour in contours:

            area = cv2.contourArea(contour)

            # Strict area limits. Ores are small but visible.
            if area < 150 or area > 6000:
                continue

            x, y, w, h = cv2.boundingRect(
                contour
            )

            if w < 8 or h < 8:
                continue

            # Ore faces are roughly square.
            aspect = max(w, h) / max(min(w, h), 1)

            if aspect > 1.8:
                continue

            valid.append(
                contour
            )

        # ---------------------------------------------------------------------
        # Exactly two ores of each type.
        # Keep two largest regions.
        # ---------------------------------------------------------------------

        valid.sort(
            key=cv2.contourArea,
            reverse=True
        )

        valid = valid[:2]

        # ---------------------------------------------------------------------
        # Find contour centre
        # ---------------------------------------------------------------------

        for contour in valid:

            M = cv2.moments(
                contour
            )

            if M['m00'] == 0:
                continue

            cX = int(
                M['m10'] / M['m00']
            )

            cY = int(
                M['m01'] / M['m00']
            )

            center_ore_list.append(
                (cX, cY)
            )

            ore_type_list.append(
                ore_type
            )

            contour_list.append(
                contour
            )

    return (
        center_ore_list,
        ore_type_list,
        contour_list
    )


# =============================================================================
# NODE
# =============================================================================

class ore_tf(Node):

    def __init__(self):

        super().__init__(
            'ore_tf_publisher'
        )

        # ---------------------------------------------------------------------
        # Subscribers
        # ---------------------------------------------------------------------

        self.color_cam_sub = self.create_subscription(
            Image,
            color_topic,
            self.colorimagecb,
            10
        )

        self.depth_cam_sub = self.create_subscription(
            Image,
            depth_topic,
            self.depthimagecb,
            10
        )

        self.cam_info_sub = self.create_subscription(
            CameraInfo,
            camera_info_topic,
            self.caminfocb,
            10
        )

        # ---------------------------------------------------------------------
        # CvBridge
        # ---------------------------------------------------------------------

        self.bridge = CvBridge()

        # ---------------------------------------------------------------------
        # TF listener
        # ---------------------------------------------------------------------

        self.tf_buffer = tf2_ros.Buffer()

        self.listener = tf2_ros.TransformListener(
            self.tf_buffer,
            self
        )

        # ---------------------------------------------------------------------
        # TF broadcaster
        # ---------------------------------------------------------------------

        self.br = tf2_ros.TransformBroadcaster(
            self
        )

        # ---------------------------------------------------------------------
        # Image processing timer
        # ---------------------------------------------------------------------

        image_processing_rate = 0.2

        self.timer = self.create_timer(
            image_processing_rate,
            self.process_image
        )

        # ---------------------------------------------------------------------
        # Data
        # ---------------------------------------------------------------------

        self.cv_image = None
        self.depth_image = None
        self.cam_info = None

        self.color_frame_id = None

        # ---------------------------------------------------------------------
        # Persistent IDs
        #
        # Each ore type has two identities:
        #
        #     ore_1
        #     ore_2
        #
        # They are kept between frames.
        # ---------------------------------------------------------------------

        self.previous_centers = {

            'azurite_ore': {
                1: None,
                2: None
            },

            'malachite_ore': {
                1: None,
                2: None
            },

            'vanadinite_ore': {
                1: None,
                2: None
            }
        }

        # ---------------------------------------------------------------------
        # Number of detected ores for diagnostics
        # ---------------------------------------------------------------------

        self.last_count = 0

        # ---------------------------------------------------------------------
        # Annotated image
        # ---------------------------------------------------------------------

        self.output_path = os.path.expanduser(
            '~/ros2_ws/src/SC#5466_task1A_detection.png'
        )

        self.get_logger().info(
            'Task 1A Ore TF Publisher started.'
        )

    # =========================================================================
    # DEPTH CALLBACK
    # =========================================================================

    def depthimagecb(self, data):

        try:

            self.depth_image = self.bridge.imgmsg_to_cv2(
                data,
                desired_encoding='passthrough'
            )

        except CvBridgeError as e:

            self.get_logger().error(
                f'Depth image conversion failed: {e}'
            )

    # =========================================================================
    # COLOR CALLBACK
    # =========================================================================

    def colorimagecb(self, data):

        try:

            self.cv_image = self.bridge.imgmsg_to_cv2(
                data,
                desired_encoding='bgr8'
            )

            self.color_frame_id = data.header.frame_id

        except CvBridgeError as e:

            self.get_logger().error(
                f'Colour image conversion failed: {e}'
            )

    # =========================================================================
    # CAMERA INFO CALLBACK
    # =========================================================================

    def caminfocb(self, data):

        self.cam_info = data

    # =========================================================================
    # DEPTH FUNCTION
    # =========================================================================

    def get_depth(self, cX, cY):

        if self.depth_image is None:
            return None

        height, width = self.depth_image.shape[:2]

        # ---------------------------------------------------------------------
        # Ensure pixel lies inside image
        # ---------------------------------------------------------------------

        if cX < 0 or cX >= width:
            return None

        if cY < 0 or cY >= height:
            return None

        # ---------------------------------------------------------------------
        # 9 x 9 window
        # ---------------------------------------------------------------------

        x1 = max(
            0,
            cX - 4
        )

        x2 = min(
            width,
            cX + 5
        )

        y1 = max(
            0,
            cY - 4
        )

        y2 = min(
            height,
            cY + 5
        )

        patch = self.depth_image[
            y1:y2,
            x1:x2
        ]

        patch = np.asarray(
            patch,
            dtype=np.float32
        )

        # ---------------------------------------------------------------------
        # Convert depth to metres.
        #
        # 16UC1 -> mm
        # 32FC1 -> m
        # ---------------------------------------------------------------------

        if self.depth_image.dtype == np.uint16:

            patch = patch / 1000.0

        # ---------------------------------------------------------------------
        # Valid depth
        # ---------------------------------------------------------------------

        valid = patch[
            np.isfinite(patch) &
            (patch > 0.05) &
            (patch < 5.0)
        ]

        if valid.size == 0:
            return None

        # Median gives a stable measurement
        z = np.median(
            valid
        )

        return float(z)

    # =========================================================================
    # DISTANCE
    # =========================================================================

    @staticmethod
    def pixel_distance(a, b):

        if a is None or b is None:

            return 1.0e12

        dx = float(
            a[0] - b[0]
        )

        dy = float(
            a[1] - b[1]
        )

        return (
            dx * dx +
            dy * dy
        )

    # =========================================================================
    # ID ASSIGNMENT
    # =========================================================================

    def assign_ids(self, detections):

        result = {}

        for ore_type in ore_types:

            current = list(
                detections.get(
                    ore_type,
                    []
                )
            )

            # ---------------------------------------------------------------
            # No detection
            # ---------------------------------------------------------------

            if len(current) == 0:

                result[ore_type] = []

                continue

            # ---------------------------------------------------------------
            # Maximum two ores
            # ---------------------------------------------------------------

            current = current[:2]

            previous = self.previous_centers[
                ore_type
            ]

            # ---------------------------------------------------------------
            # First frame
            #
            # Use deterministic left-to-right ordering.
            # ---------------------------------------------------------------

            if (
                previous[1] is None and
                previous[2] is None
            ):

                current.sort(
                    key=lambda p: (
                        p[0],
                        p[1]
                    )
                )

                assigned = []

                for i, center in enumerate(
                    current,
                    start=1
                ):

                    assigned.append(
                        (
                            i,
                            center[0],
                            center[1]
                        )
                    )

            # ---------------------------------------------------------------
            # Previous positions exist
            # ---------------------------------------------------------------

            else:

                assigned = []

                if len(current) == 1:

                    center = current[0]

                    d1 = self.pixel_distance(
                        center,
                        previous[1]
                    )

                    d2 = self.pixel_distance(
                        center,
                        previous[2]
                    )

                    if (
                        previous[1] is None
                        and previous[2] is not None
                    ):

                        chosen_id = 1

                    elif (
                        previous[2] is None
                        and previous[1] is not None
                    ):

                        chosen_id = 2

                    elif d1 <= d2:

                        chosen_id = 1

                    else:

                        chosen_id = 2

                    assigned.append(
                        (
                            chosen_id,
                            center[0],
                            center[1]
                        )
                    )

                else:

                    # Two current detections
                    a = current[0]
                    b = current[1]

                    # Option A:
                    # a -> ID1
                    # b -> ID2
                    cost_a = (
                        self.pixel_distance(
                            a,
                            previous[1]
                        )
                        +
                        self.pixel_distance(
                            b,
                            previous[2]
                        )
                    )

                    # Option B:
                    # a -> ID2
                    # b -> ID1
                    cost_b = (
                        self.pixel_distance(
                            a,
                            previous[2]
                        )
                        +
                        self.pixel_distance(
                            b,
                            previous[1]
                        )
                    )

                    if cost_a <= cost_b:

                        assigned = [
                            (
                                1,
                                a[0],
                                a[1]
                            ),
                            (
                                2,
                                b[0],
                                b[1]
                            )
                        ]

                    else:

                        assigned = [
                            (
                                2,
                                a[0],
                                a[1]
                            ),
                            (
                                1,
                                b[0],
                                b[1]
                            )
                        ]

            # ---------------------------------------------------------------
            # Store new positions
            # ---------------------------------------------------------------

            for ore_id, x, y in assigned:

                previous[ore_id] = (
                    x,
                    y
                )

            result[ore_type] = assigned

        return result

    # =========================================================================
    # PROCESS IMAGE
    # =========================================================================

    def process_image(self):

        # ---------------------------------------------------------------------
        # Wait until all camera information exists
        # ---------------------------------------------------------------------

        if self.cv_image is None:
            return

        if self.depth_image is None:
            return

        if self.cam_info is None:
            return

        if self.color_frame_id is None:
            return

        # ---------------------------------------------------------------------
        # Detect ores
        # ---------------------------------------------------------------------

        try:

            centers, ore_type_list, contour_list = \
                detect_ores(
                    self.cv_image
                )

        except Exception as e:

            self.get_logger().error(
                f'Ore detection error: {e}'
            )

            return

        self.last_count = len(
            centers
        )

        # ---------------------------------------------------------------------
        # Annotated image
        # ---------------------------------------------------------------------

        annotated = self.cv_image.copy()

        # ---------------------------------------------------------------------
        # Organise detections
        # ---------------------------------------------------------------------

        detections = {

            'azurite_ore': [],
            'malachite_ore': [],
            'vanadinite_ore': []
        }

        contours_by_position = {}

        for i in range(
            len(centers)
        ):

            center = centers[i]
            ore_type = ore_type_list[i]
            contour = contour_list[i]

            detections[
                ore_type
            ].append(
                center
            )

            contours_by_position[
                (
                    ore_type,
                    center
                )
            ] = contour

        # ---------------------------------------------------------------------
        # Persistent IDs
        # ---------------------------------------------------------------------

        identified = self.assign_ids(
            detections
        )

        # ---------------------------------------------------------------------
        # Camera intrinsics
        #
        # K =
        #
        # [ fx  0  cx ]
        # [ 0  fy  cy ]
        # [ 0   0   1 ]
        # ---------------------------------------------------------------------

        fx = float(
            self.cam_info.k[0]
        )

        fy = float(
            self.cam_info.k[4]
        )

        cx = float(
            self.cam_info.k[2]
        )

        cy = float(
            self.cam_info.k[5]
        )

        if fx == 0.0 or fy == 0.0:

            self.get_logger().error(
                'Invalid camera intrinsics.'
            )

            return

        # ---------------------------------------------------------------------
        # Camera optical frame -> base_link
        # ---------------------------------------------------------------------

        try:

            camera_to_base = \
                self.tf_buffer.lookup_transform(
                    base_frame,
                    self.color_frame_id,
                    rclpy.time.Time()
                )

        except Exception as e:

            self.get_logger().warn(
                f'TF not available yet: '
                f'{self.color_frame_id} -> '
                f'{base_frame}'
            )

            return

        # ---------------------------------------------------------------------
        # Process every ore
        # ---------------------------------------------------------------------

        published = 0

        for ore_type in ore_types:

            for ore_id, cX, cY in identified[
                ore_type
            ]:

                # -------------------------------------------------------------
                # Get depth
                # -------------------------------------------------------------

                depth = self.get_depth(
                    cX,
                    cY
                )

                if depth is None:

                    self.get_logger().warn(
                        f'Invalid depth at '
                        f'({cX}, {cY}) for '
                        f'{ore_type}_{ore_id}'
                    )

                    continue

                # -------------------------------------------------------------
                # Pixel -> camera optical coordinates
                # -------------------------------------------------------------

                x_cam = (
                    (float(cX) - cx)
                    * depth
                    / fx
                )

                y_cam = (
                    (float(cY) - cy)
                    * depth
                    / fy
                )

                z_cam = float(
                    depth
                )

                # -------------------------------------------------------------
                # Create PointStamped
                # -------------------------------------------------------------

                point_camera = PointStamped()

                point_camera.header.stamp = \
                    self.get_clock().now().to_msg()

                point_camera.header.frame_id = \
                    self.color_frame_id

                point_camera.point.x = float(
                    x_cam
                )

                point_camera.point.y = float(
                    y_cam
                )

                point_camera.point.z = float(
                    z_cam
                )

                # -------------------------------------------------------------
                # Transform point into base_link
                # -------------------------------------------------------------

                try:

                    point_base = \
                        do_transform_point(
                            point_camera,
                            camera_to_base
                        )

                except Exception as e:

                    self.get_logger().error(
                        f'Point transformation failed: '
                        f'{e}'
                    )

                    continue

                # -------------------------------------------------------------
                # TOP FACE -> CENTRE OF ORE
                #
                # Camera sees top face.
                # Ore centre lies 0.0381 m below it.
                #
                # NOTE:
                # The correction is applied after transformation to base_link
                # because the competition's base_link z-axis is vertical.
                # -------------------------------------------------------------

                x_base = float(
                    point_base.point.x
                )

                y_base = float(
                    point_base.point.y
                )

                z_base = float(
                    point_base.point.z
                )

                # -------------------------------------------------------------
                # Frame name
                # -------------------------------------------------------------

                child_frame = (
                    f'{ore_type}_{ore_id}'
                )

                # -------------------------------------------------------------
                # TransformStamped
                # -------------------------------------------------------------

                t = TransformStamped()

                t.header.stamp = \
                    self.get_clock().now().to_msg()

                t.header.frame_id = \
                    base_frame

                t.child_frame_id = \
                    child_frame

                # -------------------------------------------------------------
                # Translation
                # -------------------------------------------------------------

                t.transform.translation.x = float(
                    x_base
                )

                t.transform.translation.y = float(
                    y_base
                )

                t.transform.translation.z = float(
                    z_base
                )

                # -------------------------------------------------------------
                # Identity quaternion
                #
                # Required even though orientation is not scored.
                # -------------------------------------------------------------

                t.transform.rotation.x = 0.0
                t.transform.rotation.y = 0.0
                t.transform.rotation.z = 0.0
                t.transform.rotation.w = 1.0

                # -------------------------------------------------------------
                # Broadcast
                # -------------------------------------------------------------

                self.br.sendTransform(
                    t
                )

                published += 1

                # -------------------------------------------------------------
                # Find contour for annotation
                # -------------------------------------------------------------

                contour = contours_by_position.get(
                    (
                        ore_type,
                        (cX, cY)
                    ),
                    None
                )

                if contour is not None:

                    cv2.drawContours(
                        annotated,
                        [contour],
                        -1,
                        (255, 255, 255),
                        2
                    )

                # -------------------------------------------------------------
                # Draw centre
                # -------------------------------------------------------------

                cv2.circle(
                    annotated,
                    (cX, cY),
                    5,
                    (255, 255, 255),
                    -1
                )

                # -------------------------------------------------------------
                # Label
                # -------------------------------------------------------------

                label = child_frame

                text_x = max(
                    5,
                    cX - 70
                )

                text_y = max(
                    25,
                    cY - 10
                )

                cv2.putText(
                    annotated,
                    label,
                    (text_x, text_y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (255, 255, 255),
                    2,
                    cv2.LINE_AA
                )

                # -------------------------------------------------------------
                # Log
                # -------------------------------------------------------------

                self.get_logger().info(
                    f'{child_frame}: '
                    f'x={x_base:.4f}, '
                    f'y={y_base:.4f}, '
                    f'z={z_base:.4f}'
                )

        # ---------------------------------------------------------------------
        # Save annotated image
        # ---------------------------------------------------------------------

        cv2.imwrite(
            self.output_path,
            annotated
        )

        # ---------------------------------------------------------------------
        # Helpful diagnostic
        # ---------------------------------------------------------------------

        if published == 6:

            self.get_logger().info(
                'Successfully published all 6 ore transforms.'
            )

        else:

            self.get_logger().warn(
                f'Published {published}/6 ore transforms.'
            )


# =============================================================================
# MAIN
# =============================================================================

def main(args=None):

    rclpy.init(
        args=args
    )

    node = ore_tf()

    try:

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()


# =============================================================================
# PROGRAM ENTRY
# =============================================================================

if __name__ == '__main__':

    main()
