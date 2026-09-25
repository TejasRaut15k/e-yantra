#!/usr/bin/env python3
"""
Task 1B – UR7e Arm Waypoint Navigation
State machine: LIFT → ROTATE → ORIENT → TRANSLATE → DROP → HOLD → next WP

Key insight: orientation must be set BEFORE horizontal translation begins,
so the elbow never passes through q[2]=0 (singularity). The ORIENT phase
tilts the tool outward while the arm is still retracted, then TRANSLATE
moves horizontally with the wrist already extended.
"""

import rclpy
import sys
import math
import numpy as np
from rclpy.node import Node
from control_msgs.msg import JointJog
from controller_manager_msgs.srv import SwitchController
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32
from scipy.spatial.transform import Rotation

# ─── Task constants ───────────────────────────────────────────────
waypoints = [
    (-0.4085, -0.5379, 0.1967),   # WP1  r=0.675
    (-0.8000, -0.0005, 0.3967),   # WP2  r=0.800
    (-0.7430,  0.5280, 0.1967),   # WP3  r=0.911
    (-0.4097,  0.5280, 0.1967),   # WP4  r=0.669
    (-0.0763,  0.5280, 0.1967),   # WP5  r=0.533
]

servo_ns        = '/ur_arm_controller'
twist_ctrl_name = 'delta_twist_controller'
joint_ctrl_name = 'delta_joint_controller'
base_frame      = 'base_link'

joint_names = [
    'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint',
]

# Servo hard caps (from boilerplate). Commands ABOVE these are DROPPED WHOLE.
CAP_LINEAR  = 0.15   # m/s
CAP_ANGULAR = 0.35   # rad/s
CAP_JOINT   = 0.35   # rad/s

# Our soft limits (safety margin below hard caps)
MAX_LIN  = 0.14      # m/s
MAX_ANG  = 0.25      # rad/s
MAX_PAN  = 0.28      # rad/s

KP_XYZ       = 1.0
KP_PAN       = 1.0
KP_ORIENT    = 0.8
TOLERANCE    = 0.012  # m – arrival threshold
HOLD_DURATION = 2.5   # s – hold longer than 2s for safety
LIFT_Z       = 0.55   # m – clearance altitude


def _orient_dir_for_wp(tx, ty):
    """
    Return the desired tool-Z direction for a given waypoint.
    Far waypoints need outward tilt to avoid elbow singularity.
    """
    r = math.sqrt(tx*tx + ty*ty)
    if r > 0.78:
        # Strong outward tilt for far reach (WP2 r=0.80, WP3 r=0.91)
        return np.array([tx, ty, -0.3])
    elif r > 0.60:
        # Gentle outward tilt (WP1 r=0.675, WP4 r=0.669)
        return np.array([tx, ty, -1.0])
    else:
        # Straight down (WP5 r=0.533)
        return np.array([0.0, 0.0, -1.0])


class ArmController(Node):

    def __init__(self):
        super().__init__(
            'arm_waypoints_node',
            parameter_overrides=[
                rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)
            ])

        # Publishers
        self.twist_pub = self.create_publisher(TwistStamped, '/delta_twist_cmds', 10)
        self.joint_pub = self.create_publisher(JointJog, '/delta_joint_cmds', 10)

        # Subscribers
        self.create_subscription(PoseStamped, '/tcp_pose_raw', self._tcp_cb, 20)
        self.create_subscription(JointState,  '/joint_states',  self._js_cb,  50)
        self.create_subscription(Int32,       '/arm_status',    self._st_cb,  10)

        # Controller switch service
        self.switch_cli = self.create_client(SwitchController, f'{servo_ns}/switch_controller')

        # Feedback
        self.tcp_pose    = None
        self.joint_dict  = {}
        self.arm_status  = 0

        # State machine
        self.wp_idx      = 0
        self.state       = 'LIFT'
        self.hold_start  = None
        self.active_ctrl = twist_ctrl_name
        self.switching   = False
        self.switch_future = None

        # Logging
        self.tick = 0
        self.results = []

        self.timer = self.create_timer(0.05, self._control)
        self.get_logger().info('=== Task 1B Controller (6-state) Started ===')

    # ── Callbacks ──────────────────────────────────────────────────
    def _tcp_cb(self, msg):
        self.tcp_pose = msg.pose

    def _js_cb(self, msg):
        for i, name in enumerate(msg.name):
            self.joint_dict[name] = msg.position[i]

    def _st_cb(self, msg):
        self.arm_status = msg.data

    # ── Controller switching (poll-based, safe from timer callback) ──
    def _switch(self, want):
        if self.active_ctrl == want:
            return
        self.switching = True
        req = SwitchController.Request()
        if want == twist_ctrl_name:
            req.activate_controllers = [twist_ctrl_name]
            req.deactivate_controllers = [joint_ctrl_name]
        else:
            req.activate_controllers = [joint_ctrl_name]
            req.deactivate_controllers = [twist_ctrl_name]
        req.strictness = SwitchController.Request.BEST_EFFORT
        self.switch_future = self.switch_cli.call_async(req)

    def _check_switch(self):
        if not self.switching:
            return False
        if self.switch_future is None:
            self.switching = False
            return False
        if not self.switch_future.done():
            return True
        result = self.switch_future.result()
        if result and result.ok:
            if self.active_ctrl == twist_ctrl_name:
                self.active_ctrl = joint_ctrl_name
            else:
                self.active_ctrl = twist_ctrl_name
            self.get_logger().info(f'Switched to {self.active_ctrl}')
        else:
            self.get_logger().warn('Switch failed, will retry')
        self.switching = False
        self.switch_future = None
        return False

    # ── Command helpers ───────────────────────────────────────────
    def _pub_twist(self, vx, vy, vz, wx=0.0, wy=0.0, wz=0.0):
        lin = math.sqrt(vx*vx + vy*vy + vz*vz)
        if lin > MAX_LIN:
            s = MAX_LIN / lin
            vx, vy, vz = vx*s, vy*s, vz*s
        ang = math.sqrt(wx*wx + wy*wy + wz*wz)
        if ang > MAX_ANG:
            s = MAX_ANG / ang
            wx, wy, wz = wx*s, wy*s, wz*s
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = base_frame
        msg.twist.linear.x  = float(vx)
        msg.twist.linear.y  = float(vy)
        msg.twist.linear.z  = float(vz)
        msg.twist.angular.x = float(wx)
        msg.twist.angular.y = float(wy)
        msg.twist.angular.z = float(wz)
        self.twist_pub.publish(msg)

    def _pub_joint(self, velocities):
        msg = JointJog()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = base_frame
        msg.joint_names = list(joint_names)
        msg.velocities = [float(max(-CAP_JOINT, min(CAP_JOINT, v))) for v in velocities]
        self.joint_pub.publish(msg)

    # ── Orientation P-controller ──────────────────────────────────
    def _orient_error(self, target_dir):
        """Return (wx, wy, wz, angle_err) to align tool Z with target_dir."""
        try:
            q = self.tcp_pose.orientation
            R = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
            a = R[:, 2]
            b = np.array(target_dir, dtype=float)
            bn = np.linalg.norm(b)
            if bn < 1e-6:
                return 0.0, 0.0, 0.0, 0.0
            b = b / bn
            c = np.cross(a, b)
            s = np.linalg.norm(c)
            d = np.dot(a, b)
            angle = math.atan2(s, d)
            if s < 1e-6:
                return 0.0, 0.0, 0.0, angle
            e = c / s * angle
            return KP_ORIENT * e[0], KP_ORIENT * e[1], KP_ORIENT * e[2], angle
        except Exception:
            return 0.0, 0.0, 0.0, 0.0

    # ── Main control loop ─────────────────────────────────────────
    def _control(self):
        if self.tcp_pose is None or len(self.joint_dict) < 6:
            return
        if self._check_switch():
            self._pub_twist(0, 0, 0)
            return

        self.tick += 1

        if self.state == 'DONE':
            self._pub_twist(0, 0, 0)
            return

        cx = self.tcp_pose.position.x
        cy = self.tcp_pose.position.y
        cz = self.tcp_pose.position.z
        tx, ty, tz = waypoints[self.wp_idx]
        ex, ey, ez_wp = tx - cx, ty - cy, tz - cz
        dist3d = math.sqrt(ex*ex + ey*ey + ez_wp*ez_wp)

        orient_dir = _orient_dir_for_wp(tx, ty)

        # ─── LIFT: go up to clearance altitude ────────────────────
        if self.state == 'LIFT':
            if self.active_ctrl != twist_ctrl_name:
                self._switch(twist_ctrl_name)
                return
            ez = LIFT_Z - cz
            if abs(ez) < TOLERANCE:
                self._pub_twist(0, 0, 0)
                self.state = 'ROTATE'
                self.get_logger().info(f'[WP{self.wp_idx+1}] LIFT done → ROTATE')
            else:
                if self.wp_idx == 0:
                    # Pure Z, no orientation for the very first lift to avoid spawn singularity
                    self._pub_twist(0, 0, KP_XYZ * ez)
                else:
                    # For subsequent lifts, if the arm is far out (r > 0.75), 
                    # retract it radially towards the base while lifting.
                    # This safely bends the elbow and completely avoids the singularity
                    # that occurs when lifting straight up at full extension.
                    r_curr = math.sqrt(cx*cx + cy*cy)
                    vx = 0.0
                    vy = 0.0
                    if r_curr > 0.75:
                        vx = -cx * KP_XYZ * 0.5
                        vy = -cy * KP_XYZ * 0.5
                    
                    # Keep pointing straight down during lift
                    lift_orient = [0.0, 0.0, -1.0]
                    owx, owy, owz, _ = self._orient_error(lift_orient)
                    self._pub_twist(vx, vy, KP_XYZ * ez, owx, owy, owz)

                if self.tick % 20 == 0:
                    self.get_logger().info(
                        f'[WP{self.wp_idx+1}] LIFT z={cz:.3f} err={ez:.3f} status={self.arm_status}')

        # ─── ROTATE: swing shoulder pan to face the waypoint ──────
        elif self.state == 'ROTATE':
            if self.active_ctrl != joint_ctrl_name:
                self._switch(joint_ctrl_name)
                return
            target_pan = math.atan2(ty, tx)
            current_pan = self.joint_dict.get('shoulder_pan_joint', 0.0)
            err = target_pan - current_pan
            while err > math.pi:  err -= 2 * math.pi
            while err < -math.pi: err += 2 * math.pi

            if abs(err) < 0.02:
                self._pub_joint([0]*6)
                self.state = 'ORIENT'
                self.get_logger().info(f'[WP{self.wp_idx+1}] ROTATE done → ORIENT')
            else:
                vel = max(-MAX_PAN, min(MAX_PAN, KP_PAN * err))
                self._pub_joint([vel, 0, 0, 0, 0, 0])
                if self.tick % 20 == 0:
                    self.get_logger().info(
                        f'[WP{self.wp_idx+1}] ROTATE pan_err={math.degrees(err):.1f}° status={self.arm_status}')

        # ─── ORIENT: tilt tool to target orientation BEFORE translating ──
        elif self.state == 'ORIENT':
            if self.active_ctrl != twist_ctrl_name:
                self._switch(twist_ctrl_name)
                return
            owx, owy, owz, angle_err = self._orient_error(orient_dir)
            if angle_err < 0.08:  # ~4.5 degrees
                self._pub_twist(0, 0, 0)
                self.state = 'TRANSLATE'
                self.get_logger().info(f'[WP{self.wp_idx+1}] ORIENT done (err={math.degrees(angle_err):.1f}°) → TRANSLATE')
            else:
                # Only orient, no translation — keep arm retracted and safe
                self._pub_twist(0, 0, 0, owx, owy, owz)
                if self.tick % 20 == 0:
                    self.get_logger().info(
                        f'[WP{self.wp_idx+1}] ORIENT angle_err={math.degrees(angle_err):.1f}° status={self.arm_status}')

        # ─── TRANSLATE: move horizontally to waypoint XY ──────────
        elif self.state == 'TRANSLATE':
            if self.active_ctrl != twist_ctrl_name:
                self._switch(twist_ctrl_name)
                return
            horiz = math.sqrt(ex*ex + ey*ey)
            if horiz < TOLERANCE:
                self._pub_twist(0, 0, 0)
                self.state = 'DROP'
                self.get_logger().info(f'[WP{self.wp_idx+1}] TRANSLATE done → DROP')
            else:
                # Maintain orientation during translate to keep the reach
                owx, owy, owz, _ = self._orient_error(orient_dir)
                self._pub_twist(
                    KP_XYZ * ex, KP_XYZ * ey, KP_XYZ * (LIFT_Z - cz),
                    owx, owy, owz)
                if self.tick % 20 == 0:
                    elbow = self.joint_dict.get('elbow_joint', 0.0)
                    self.get_logger().info(
                        f'[WP{self.wp_idx+1}] TRANSLATE dist={horiz:.3f} elbow={math.degrees(elbow):.1f}° status={self.arm_status}')

        # ─── DROP: descend to waypoint Z ──────────────────────────
        elif self.state == 'DROP':
            if abs(ez_wp) < TOLERANCE:
                self._pub_twist(0, 0, 0)
                self.hold_start = self.get_clock().now()
                self.state = 'HOLD'
                self.get_logger().info(
                    f'[WP{self.wp_idx+1}] DROP done → HOLD  pos=({cx:.4f},{cy:.4f},{cz:.4f})')
            else:
                # During DROP, transition toward straight-down to let elbow bend
                # and avoid singularity at full extension. Also allow XY drift
                # correction so we stay on target as the arm curls inward.
                drop_orient = [0.0, 0.0, -1.0]
                owx, owy, owz, _ = self._orient_error(drop_orient)
                self._pub_twist(KP_XYZ * ex, KP_XYZ * ey, KP_XYZ * ez_wp, owx, owy, owz)
                if self.tick % 20 == 0:
                    self.get_logger().info(
                        f'[WP{self.wp_idx+1}] DROP z_err={ez_wp:.3f} xy_err={math.sqrt(ex*ex+ey*ey):.3f} status={self.arm_status}')

        # ─── HOLD: command zero velocity for 2.5 seconds ──────────
        elif self.state == 'HOLD':
            self._pub_twist(0, 0, 0)
            elapsed = (self.get_clock().now() - self.hold_start).nanoseconds / 1e9
            if self.tick % 10 == 0:
                self.get_logger().info(
                    f'[WP{self.wp_idx+1}] HOLD err={dist3d:.4f}m t={elapsed:.1f}/{HOLD_DURATION:.1f}s status={self.arm_status}')

            if elapsed >= HOLD_DURATION:
                self.results.append({
                    'wp': self.wp_idx + 1,
                    'target': (tx, ty, tz),
                    'actual': (cx, cy, cz),
                    'error': dist3d,
                    'hold_s': elapsed,
                })
                self.get_logger().info(
                    f'✅ WP{self.wp_idx+1} REACHED | '
                    f'target=({tx:.4f},{ty:.4f},{tz:.4f}) | '
                    f'actual=({cx:.4f},{cy:.4f},{cz:.4f}) | '
                    f'error={dist3d:.4f}m | hold={elapsed:.1f}s')

                self.wp_idx += 1
                if self.wp_idx >= len(waypoints):
                    self.get_logger().info('🎯 ═══ ALL 5 WAYPOINTS REACHED ═══')
                    self._print_summary()
                    self.state = 'DONE'
                else:
                    self.state = 'LIFT'

    def _print_summary(self):
        self.get_logger().info('═══════════ FINAL RESULTS ═══════════')
        all_pass = True
        for r in self.results:
            ok = '✅' if r['error'] <= 0.03 else '❌'
            if r['error'] > 0.03:
                all_pass = False
            self.get_logger().info(
                f"  WP{r['wp']} {ok} err={r['error']:.4f}m hold={r['hold_s']:.1f}s "
                f"actual=({r['actual'][0]:.4f},{r['actual'][1]:.4f},{r['actual'][2]:.4f})")
        if all_pass:
            self.get_logger().info('🏆 ALL WAYPOINTS WITHIN 0.03m BONUS THRESHOLD')
        else:
            self.get_logger().info('⚠️  Some waypoints exceeded 0.03m threshold')
        self.get_logger().info('═════════════════════════════════════')


def main():
    rclpy.init(args=sys.argv)
    node = ArmController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
