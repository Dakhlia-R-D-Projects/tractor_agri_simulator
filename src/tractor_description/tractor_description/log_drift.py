#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import pandas as pd
import os
from threading import Lock
from math import sqrt

class DriftLogger(Node):
    def __init__(self):
        super().__init__('drift_logger')

        # Parameter for CSV output path
        self.declare_parameter('csv_path', 'drift_log.csv')
        self.csv_path = self.get_parameter('csv_path').get_parameter_value().string_value

        # In-memory storage
        self._data = []
        self._lock = Lock()

        # Latest messages
        self._latest_odom = None
        self._latest_gt   = None

        # Subscribers
        self.create_subscription(Odometry, '/odometry/filtered', self._odom_cb, 10)
        self.create_subscription(Odometry, '/gt_pose_no_drift', self._gt_cb, 10)

        self.get_logger().info(f"DriftLogger initialized; will save to '{self.csv_path}' on shutdown.")

    def _odom_cb(self, odom_msg: Odometry):
        self._latest_odom = odom_msg
        self._record_if_possible(source='odom')

    def _gt_cb(self, gt_msg: Odometry):
        self._latest_gt = gt_msg
        self._record_if_possible(source='gt')

    def _record_if_possible(self, source: str):
        """
        Attempt to record a row whenever either message arrives.
        Uses the newest of one and the most recent of the other.
        """
        if self._latest_odom is None or self._latest_gt is None:
            # still waiting for the first pair
            return

        # extract positions
        o = self._latest_odom.pose.pose.position
        g = self._latest_gt.pose.pose.position

        dx = o.x - g.x
        dy = o.y - g.y
        dz = o.z - g.z
        dist = sqrt(dx*dx + dy*dy + dz*dz)

        ts = self._latest_odom.header.stamp
        ros_time = f"{ts.sec}.{ts.nanosec:09d}"

        row = {
            'ros_time':   ros_time,
            'odom_x':     o.x,      'odom_y':   o.y,      'odom_z': o.z,
            'gt_x':       g.x,      'gt_y':     g.y,      'gt_z':   g.z,
            'drift_dx':   dx,       'drift_dy': dy,       'drift_dz': dz,
            'drift_dist': dist,
            'source':     source      # optional: which callback triggered this
        }

        with self._lock:
            self._data.append(row)

    def destroy_node(self):
        # On shutdown, dump to pandas DataFrame and save CSV
        with self._lock:
            df = pd.DataFrame(self._data)

        # ensure directory exists
        os.makedirs(os.path.dirname(self.csv_path) or '.', exist_ok=True)
        df.to_csv(self.csv_path, index=False)
        self.get_logger().info(f"Wrote {len(df)} rows to '{self.csv_path}'")

        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = DriftLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
