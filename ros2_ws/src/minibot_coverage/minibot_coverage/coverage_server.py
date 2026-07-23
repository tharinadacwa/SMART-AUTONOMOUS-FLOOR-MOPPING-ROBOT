#!/usr/bin/env python3
"""
coverage_server.py -- executes the pre-computed coverage path through Nav2.

WHO CALLS IT      minibot_bringup/clean.launch.py
WHAT IT NEEDS     a *_coverage_path.yaml produced OFFLINE by prepare_map.py
                  Nav2 up and ACTIVE (navigate_through_poses action available)
                  AMCL localized (give a 2D Pose Estimate first)
WHAT IT PUBLISHES /coverage_path      (nav_msgs/Path,        latched)
                  /coverage_map       (nav_msgs/OccupancyGrid, latched)
                  /coverage_waypoints (visualization_msgs/MarkerArray, latched)
                                      GREEN=pending RED=current GREY=done YELLOW=failed
                  /coverage_progress  (std_msgs/Float32, 0..100)
                  /diagnostics
SERVICES          /coverage/start  /coverage/stop  /coverage/skip  /coverage/dock

DELIBERATELY DUMB
    All the thinking happened OFFLINE, in prepare_map.py, where a mistake costs
    you a re-run of a Python script instead of a robot wedged behind the sofa for
    15 minutes. You already looked at the preview PNG and approved the path. This
    node's entire job is to execute exactly that -- no runtime replanning that
    could surprise you halfway through.

    The map is static, so the path is static. Computing it at runtime would buy
    nothing and cost determinism.

RESUME
    Progress is checkpointed after EVERY chunk. If the run dies at 70% -- battery,
    a crash, you tripped over it -- relaunch with resume:=true and it picks up
    from the last completed chunk instead of re-cleaning the whole flat.

RETURN TO DOCK
    On completion (or on /coverage/dock), drives to the dock pose from robot.yaml.
"""

import math
import os
from enum import Enum

import numpy as np
import rclpy
import yaml
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

from geometry_msgs.msg import PoseStamped, Quaternion, Point
from nav_msgs.msg import OccupancyGrid, Path
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from std_msgs.msg import Float32, ColorRGBA
from std_srvs.srv import Trigger
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from visualization_msgs.msg import Marker, MarkerArray

import tf2_ros


# RViz waypoint-status colors (r, g, b, a) -- same scheme as mopping_coverage8.py
_C_GREEN  = (0.00, 0.90, 0.20, 0.90)   # PENDING  -- not reached yet
_C_RED    = (1.00, 0.00, 0.00, 1.00)   # CURRENT  -- being navigated right now
_C_GREY   = (0.50, 0.50, 0.50, 0.60)   # DONE     -- covered by the robot
_C_YELLOW = (1.00, 1.00, 0.00, 0.90)   # FAILED   -- skipped / Nav2 gave up
_WP_SCALE = 0.07                        # marker sphere diameter, metres


class State(Enum):
    IDLE = 0
    CLEANING = 1
    DOCKING = 2
    DONE = 3


class CoverageServer(Node):

    def __init__(self):
        super().__init__('coverage_server')

        self.declare_parameter('path_file', '')
        self.declare_parameter('poses_per_goal', 12)
        self.declare_parameter('goal_retries', 1)
        self.declare_parameter('auto_start', False)
        self.declare_parameter('resume', False)
        self.declare_parameter('checkpoint_file', '/tmp/minibot_coverage_progress.txt')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('coverage_radius', 0.10)
        self.declare_parameter('return_to_dock', True)
        self.declare_parameter('dock_x', 0.0)
        self.declare_parameter('dock_y', 0.0)
        self.declare_parameter('dock_yaw', 0.0)

        g = self.get_parameter
        self.path_file   = g('path_file').value
        self.chunk_size  = int(g('poses_per_goal').value)
        self.retries_cfg = int(g('goal_retries').value)
        self.ckpt        = g('checkpoint_file').value
        self.map_frame   = g('map_frame').value
        self.base_frame  = g('base_frame').value
        self.cov_r       = float(g('coverage_radius').value)
        self.do_dock     = bool(g('return_to_dock').value)
        self.dock        = (float(g('dock_x').value),
                            float(g('dock_y').value),
                            float(g('dock_yaw').value))

        if not self.path_file or not os.path.exists(self.path_file):
            self.get_logger().fatal(
                f'path_file "{self.path_file}" not found.\n'
                'Generate it FIRST, offline, on your laptop:\n'
                '  ros2 run minibot_coverage prepare_map.py '
                '--map maps/home.yaml --start 0 0\n'
                'Then LOOK at maps/home_coverage_preview.png before you run this.')
            raise SystemExit(1)

        with open(self.path_file) as f:
            doc = yaml.safe_load(f)
        self.meta   = doc.get('metadata', {})
        self.poses  = doc['poses']
        self.n_edge = int(self.meta.get('n_edge_waypoints', 0))

        self.state    = State.IDLE
        self.chunk    = 0
        self.retries  = 0
        self._gh      = None
        self._auto    = False
        self.map_msg  = None
        self.visited  = None
        self.region   = None
        self.failed_chunks = 0
        self.failed_chunk_ids = set()      # chunk indices that failed -> YELLOW

        if g('resume').value and os.path.exists(self.ckpt):
            try:
                self.chunk = int(open(self.ckpt).read().strip())
                self.get_logger().warn(
                    f'RESUMING from chunk {self.chunk} '
                    f'(waypoint {self.chunk * self.chunk_size} of {len(self.poses)}). '
                    f'Delete {self.ckpt} to start fresh.')
            except Exception:
                self.chunk = 0

        cb = ReentrantCallbackGroup()
        latched = QoSProfile(depth=1,
                             reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                             history=QoSHistoryPolicy.KEEP_LAST)

        self.create_subscription(OccupancyGrid, '/map', self._on_map, latched,
                                 callback_group=cb)
        self.pub_path = self.create_publisher(Path, '/coverage_path', latched)
        self.pub_cov  = self.create_publisher(OccupancyGrid, '/coverage_map', latched)
        self.pub_prog = self.create_publisher(Float32, '/coverage_progress', 10)
        self.pub_diag = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.pub_wp   = self.create_publisher(MarkerArray, '/coverage_waypoints', latched)

        self.create_service(Trigger, '/coverage/start', self._srv_start, callback_group=cb)
        self.create_service(Trigger, '/coverage/stop',  self._srv_stop,  callback_group=cb)
        self.create_service(Trigger, '/coverage/skip',  self._srv_skip,  callback_group=cb)
        self.create_service(Trigger, '/coverage/dock',  self._srv_dock,  callback_group=cb)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.nav_through = ActionClient(self, NavigateThroughPoses,
                                        'navigate_through_poses', callback_group=cb)
        self.nav_to = ActionClient(self, NavigateToPose,
                                   'navigate_to_pose', callback_group=cb)

        self.create_timer(2.0, self._publish_path, callback_group=cb)
        self.create_timer(0.5, self._track, callback_group=cb)
        self.create_timer(1.0, self._diagnostics, callback_group=cb)
        self.create_timer(1.0, self._publish_waypoints, callback_group=cb)
        if g('auto_start').value:
            self.create_timer(10.0, self._maybe_auto, callback_group=cb)

        self.get_logger().info(
            f'Coverage server ready.\n'
            f'  path file      : {os.path.basename(self.path_file)}\n'
            f'  map            : {self.meta.get("map")}\n'
            f'  waypoints      : {len(self.poses)} ({self.n_edge} edge + '
            f'{len(self.poses) - self.n_edge} fill)\n'
            f'  lane spacing   : {self.meta.get("lane_spacing")} m\n'
            f'  path length    : {self.meta.get("path_length_m")} m\n'
            f'  estimated run  : {self.meta.get("estimated_minutes")} min\n'
            f'  planned cover  : {self.meta.get("coverage_pct")}%\n\n'
            f'  RViz: Add -> By topic -> /coverage_waypoints (MarkerArray)\n'
            f'        GREEN=pending  RED=current  GREY=done  YELLOW=failed\n\n'
            f'  START:  ros2 service call /coverage/start std_srvs/srv/Trigger')

    # ---------------- map ---------------------------------------------------

    def _on_map(self, msg):
        self.map_msg = msg
        shape = (msg.info.height, msg.info.width)
        if self.visited is None or self.visited.shape != shape:
            self.visited = np.zeros(shape, dtype=bool)
            g = np.array(msg.data, dtype=np.int8).reshape(shape)
            self.region = (g >= 0) & (g < 50)

    def _ps(self, x, y, yaw):
        p = PoseStamped()
        p.header.frame_id = self.map_frame
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x = float(x)
        p.pose.position.y = float(y)
        p.pose.orientation = Quaternion(z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))
        return p

    def _pose_of(self, d):
        return self._ps(d['x'], d['y'], d['yaw'])

    def _publish_path(self):
        m = Path()
        m.header.frame_id = self.map_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.poses = [self._pose_of(p) for p in self.poses]
        self.pub_path.publish(m)

    def _publish_waypoints(self):
        """Publish per-waypoint status spheres for RViz on /coverage_waypoints.

        Colors follow the mopping_coverage reference exactly:
            GREEN  = pending (not reached yet)
            RED    = current (the chunk being navigated now)
            GREY   = done    (already covered by the robot)
            YELLOW = failed  (skipped, or Nav2 gave up)

        One SPHERE_LIST marker carries every waypoint with a per-point color --
        this is the efficient equivalent of one sphere-per-waypoint, so it stays
        light even with tens of thousands of waypoints. A second, larger red
        sphere highlights the current target.
        """
        if not self.poses:
            return

        arr = MarkerArray()

        dots = Marker()
        dots.header.frame_id = self.map_frame
        dots.header.stamp = self.get_clock().now().to_msg()
        dots.ns = 'coverage_waypoints'
        dots.id = 0
        dots.type = Marker.SPHERE_LIST
        dots.action = Marker.ADD
        dots.scale.x = dots.scale.y = dots.scale.z = _WP_SCALE
        dots.pose.orientation.w = 1.0

        cur = self.chunk
        cleaning = (self.state == State.CLEANING)
        for i, p in enumerate(self.poses):
            ch = i // self.chunk_size
            if ch in self.failed_chunk_ids:
                c = _C_YELLOW
            elif ch < cur:
                c = _C_GREY                       # covered
            elif ch == cur and cleaning:
                c = _C_RED                        # current
            else:
                c = _C_GREEN                      # pending
            dots.points.append(Point(x=float(p['x']), y=float(p['y']), z=0.05))
            dots.colors.append(ColorRGBA(r=c[0], g=c[1], b=c[2], a=c[3]))
        arr.markers.append(dots)

        # Bigger red sphere on the current target so it is easy to spot.
        head = cur * self.chunk_size
        if cleaning and head < len(self.poses):
            hp = self.poses[head]
            cm = Marker()
            cm.header.frame_id = self.map_frame
            cm.header.stamp = self.get_clock().now().to_msg()
            cm.ns = 'coverage_current'
            cm.id = 1
            cm.type = Marker.SPHERE
            cm.action = Marker.ADD
            cm.scale.x = cm.scale.y = cm.scale.z = 0.18
            cm.color.r, cm.color.g, cm.color.b, cm.color.a = _C_RED
            cm.pose.position.x = float(hp['x'])
            cm.pose.position.y = float(hp['y'])
            cm.pose.position.z = 0.05
            cm.pose.orientation.w = 1.0
            arr.markers.append(cm)

        self.pub_wp.publish(arr)

    # ---------------- services ---------------------------------------------

    def _srv_start(self, req, resp):
        if self.state in (State.CLEANING, State.DOCKING):
            resp.success = False
            resp.message = 'Already running. /coverage/stop first.'
            return resp
        self.state = State.CLEANING
        self._send_chunk()
        resp.success = True
        resp.message = f'Cleaning from chunk {self.chunk}.'
        return resp

    def _srv_stop(self, req, resp):
        self.state = State.IDLE
        if self._gh is not None:
            self._gh.cancel_goal_async()
            self._gh = None
        self.get_logger().info(
            f'STOPPED at chunk {self.chunk}. Relaunch with resume:=true to continue.')
        resp.success = True
        resp.message = f'Stopped at chunk {self.chunk}.'
        return resp

    def _srv_skip(self, req, resp):
        """Escape hatch: the robot is wedged, move on."""
        self.get_logger().warn(f'SKIPPING chunk {self.chunk} by request.')
        if self._gh is not None:
            self._gh.cancel_goal_async()
        self.failed_chunk_ids.add(self.chunk)
        self.chunk += 1
        self.failed_chunks += 1
        self._save_ckpt()
        if self.state == State.CLEANING:
            self._send_chunk()
        resp.success = True
        resp.message = f'Skipped to chunk {self.chunk}.'
        return resp

    def _srv_dock(self, req, resp):
        self.state = State.DOCKING
        if self._gh is not None:
            self._gh.cancel_goal_async()
        self._go_dock()
        resp.success = True
        resp.message = f'Returning to dock at ({self.dock[0]}, {self.dock[1]}).'
        return resp

    def _maybe_auto(self):
        if not self._auto and self.state == State.IDLE:
            self._auto = True
            self.get_logger().info('auto_start')
            self._srv_start(Trigger.Request(), Trigger.Response())

    # ---------------- execution ---------------------------------------------

    def _save_ckpt(self):
        try:
            with open(self.ckpt, 'w') as f:
                f.write(str(self.chunk))
        except OSError as e:
            self.get_logger().warn(f'checkpoint write failed: {e}')

    def _send_chunk(self):
        if self.state != State.CLEANING:
            return

        s = self.chunk * self.chunk_size
        if s >= len(self.poses):
            self._finish_cleaning()
            return

        chunk = self.poses[s:s + self.chunk_size]

        if not self.nav_through.wait_for_server(timeout_sec=15.0):
            self.get_logger().error(
                'Nav2 navigate_through_poses is not available. Is Nav2 up and ACTIVE?\n'
                '  check:  ros2 lifecycle get /bt_navigator')
            self.state = State.IDLE
            return

        goal = NavigateThroughPoses.Goal()
        goal.poses = [self._pose_of(p) for p in chunk]

        phase = 'EDGE' if s < self.n_edge else 'FILL'
        self.get_logger().info(
            f'[{phase}] chunk {self.chunk}: wp {s}..{s + len(chunk) - 1} '
            f'of {len(self.poses)}  ({100.0 * s / len(self.poses):.0f}% issued, '
            f'{self._pct():.0f}% swept)')

        self.retries = self.retries_cfg
        self.nav_through.send_goal_async(goal).add_done_callback(self._on_resp)

    def _on_resp(self, fut):
        gh = fut.result()
        if not gh.accepted:
            self.get_logger().warn('Nav2 REJECTED the chunk; skipping it.')
            self.failed_chunk_ids.add(self.chunk)
            self.chunk += 1
            self.failed_chunks += 1
            self._save_ckpt()
            self._send_chunk()
            return
        self._gh = gh
        gh.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, fut):
        if self.state != State.CLEANING:
            return
        status = fut.result().status
        if status == 4:                                   # SUCCEEDED
            self.chunk += 1
            self._save_ckpt()
        elif self.retries > 0:
            self.retries -= 1
            self.get_logger().warn(f'chunk {self.chunk} failed ({status}); retrying')
            s = self.chunk * self.chunk_size
            goal = NavigateThroughPoses.Goal()
            goal.poses = [self._pose_of(p)
                          for p in self.poses[s:s + self.chunk_size]]
            self.nav_through.send_goal_async(goal).add_done_callback(self._on_resp)
            return
        else:
            self.get_logger().warn(
                f'chunk {self.chunk} FAILED (status {status}), retries exhausted. '
                'Skipping -- those cells will stay uncleaned.')
            self.failed_chunk_ids.add(self.chunk)
            self.chunk += 1
            self.failed_chunks += 1
            self._save_ckpt()
        self._send_chunk()

    def _finish_cleaning(self):
        pct = self._pct()
        self.get_logger().info(
            f'=== CLEANING COMPLETE: {pct:.1f}% swept, '
            f'{self.failed_chunks} chunk(s) skipped ===')
        if pct < 85.0:
            self.get_logger().warn(
                'Under 85%. Usual causes: Nav2 aborted chunks (grep the log for '
                '"FAILED"), AMCL drifted, or lane_spacing is too wide for your '
                'cleaning head.')
        try:
            os.remove(self.ckpt)
        except OSError:
            pass

        if self.do_dock:
            self.state = State.DOCKING
            self._go_dock()
        else:
            self.state = State.DONE

    def _go_dock(self):
        if not self.nav_to.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('navigate_to_pose unavailable; cannot dock.')
            self.state = State.DONE
            return
        goal = NavigateToPose.Goal()
        goal.pose = self._ps(*self.dock)
        self.get_logger().info(
            f'Returning to dock ({self.dock[0]:.2f}, {self.dock[1]:.2f})...')
        self.nav_to.send_goal_async(goal).add_done_callback(self._on_dock_resp)

    def _on_dock_resp(self, fut):
        gh = fut.result()
        if not gh.accepted:
            self.get_logger().error('Dock goal rejected.')
            self.state = State.DONE
            return
        self._gh = gh
        gh.get_result_async().add_done_callback(self._on_dock_result)

    def _on_dock_result(self, fut):
        ok = (fut.result().status == 4)
        self.get_logger().info('=== DOCKED ===' if ok else
                               '!!! FAILED TO REACH THE DOCK -- robot is parked '
                               'wherever it stopped. Recover it manually.')
        self.state = State.DONE

    # ---------------- progress + diagnostics --------------------------------

    def _track(self):
        if self.map_msg is None or self.region is None:
            return
        try:
            t = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time(),
                timeout=Duration(seconds=0.3))
        except Exception:
            return

        i = self.map_msg.info
        r0 = int((t.transform.translation.y - i.origin.position.y) / i.resolution)
        c0 = int((t.transform.translation.x - i.origin.position.x) / i.resolution)
        rad = max(1, int(round(self.cov_r / i.resolution)))
        h, w = self.visited.shape

        r_lo, r_hi = max(0, r0 - rad), min(h, r0 + rad + 1)
        c_lo, c_hi = max(0, c0 - rad), min(w, c0 + rad + 1)
        if r_lo >= r_hi or c_lo >= c_hi:
            return
        rr, cc = np.ogrid[r_lo:r_hi, c_lo:c_hi]
        self.visited[r_lo:r_hi, c_lo:c_hi] |= \
            ((rr - r0) ** 2 + (cc - c0) ** 2 <= rad * rad)

        self.pub_prog.publish(Float32(data=float(self._pct())))

        m = OccupancyGrid()
        m.header.frame_id = self.map_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.info = i
        grid = np.full(self.visited.shape, -1, dtype=np.int8)
        grid[self.region & ~self.visited] = 100     # still to clean (dark)
        grid[self.region & self.visited] = 0        # cleaned (light)
        m.data = grid.flatten().tolist()
        self.pub_cov.publish(m)

    def _pct(self):
        if self.region is None or self.visited is None:
            return 0.0
        tot = int(np.count_nonzero(self.region))
        if tot == 0:
            return 0.0
        return 100.0 * int(np.count_nonzero(self.region & self.visited)) / tot

    def _diagnostics(self):
        st = DiagnosticStatus()
        st.name = 'minibot_coverage: coverage_server'
        st.hardware_id = 'coverage'
        st.level = DiagnosticStatus.OK
        st.message = self.state.name

        if self.failed_chunks > 0:
            st.level = DiagnosticStatus.WARN
            st.message = f'{self.state.name}, {self.failed_chunks} chunk(s) skipped'

        done = min(self.chunk * self.chunk_size, len(self.poses))
        st.values = [
            KeyValue(key='state', value=self.state.name),
            KeyValue(key='chunk', value=str(self.chunk)),
            KeyValue(key='waypoints_issued', value=f'{done}/{len(self.poses)}'),
            KeyValue(key='swept_pct', value=f'{self._pct():.1f}'),
            KeyValue(key='failed_chunks', value=str(self.failed_chunks)),
        ]
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status = [st]
        self.pub_diag.publish(arr)


def main():
    rclpy.init()
    try:
        node = CoverageServer()
    except SystemExit:
        rclpy.try_shutdown()
        return
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
