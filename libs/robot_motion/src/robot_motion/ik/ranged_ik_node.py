#!/usr/bin/env python3
"""
Lightweight ROS2 node to exercise the IK code in `robot_motion/ik/` without a
full package install. Designed to be run directly from disk.

Usage:
  # in each terminal (or script) source system ROS first, e.g.:
  source /opt/ros/humble/setup.bash

  # then run robot_state_publisher with the URDF (absolute path):
  ros2 run robot_state_publisher robot_state_publisher /path/to/bimanual_arms.urdf

  # run this node (it will publish /joint_states)
  python3 /absolute/path/to/ranged_ik_node.py

This node will try to import the real `RangedIK` from `robot_motion/ik/ranged_ik.py`.
If that import fails it falls back to a simple zero-return solver so you can still
visualize the pipeline in RViz.
"""
import os
import sys
from typing import List

try:
	import rclpy
	from rclpy.node import Node
	from sensor_msgs.msg import JointState
except Exception as e:
	print("rclpy and sensor_msgs are required to run this node.\nError:", e)
	raise

# Ensure the package src directory is on sys.path so the module imports work
# when running the script directly from the repo.
_this_file = os.path.abspath(__file__)
_ik_dir = os.path.dirname(_this_file)            # .../robot_motion/ik
_pkg_src = os.path.abspath(os.path.join(_ik_dir, ".."))  # .../robot_motion
_src_root = os.path.abspath(os.path.join(_pkg_src, ".."))  # .../src
if _src_root not in sys.path:
	sys.path.insert(0, _src_root)

_HAS_RANGED_IK = False
try:
	# Preferred import when package is available on PYTHONPATH
	from robot_motion.ik.ranged_ik import RangedIK
	_HAS_RANGED_IK = True
except Exception:
	# Try a relative import if executed as a module; otherwise we'll fall back
	try:
		from .ranged_ik import RangedIK  # type: ignore
		_HAS_RANGED_IK = True
	except Exception:
		RangedIK = None
		_HAS_RANGED_IK = False


class FallbackIK:
	def __init__(self, n=7):
		self.n = n

	def solve(self, x, base_frame, ee_frame):
		return [0.0] * self.n


class RangedIKNode(Node):
	def __init__(self):
		super().__init__("ranged_ik_test_node")

		# Adjust these joint names to match your URDF if needed
		self.joint_names: List[str] = [f"left_panda_joint{i}" for i in range(1, 8)]

		self.pub = self.create_publisher(JointState, "joint_states", 10)

		# Allow passing a settings file via env var RANGED_IK_SETTINGS or CLI --settings
		import argparse
		parser = argparse.ArgumentParser(add_help=False)
		parser.add_argument("--settings", type=str, default=None)
		# parse known args to avoid interfering with rclpy args
		args, _ = parser.parse_known_args()

		settings_path = args.settings or os.environ.get("RANGED_IK_SETTINGS")

		if _HAS_RANGED_IK:
			try:
				# Pass settings_path (may be None)
				self.solver = RangedIK(settings_path=settings_path)
				self.get_logger().info("Using RangedIK from robot_motion.ik; settings=%s", str(settings_path))
			except Exception as e:
				self.get_logger().warning("Failed to init RangedIK: %s", str(e))
				self.solver = FallbackIK(n=len(self.joint_names))
		else:
			self.get_logger().warning("RangedIK not available; using fallback solver")
			self.solver = FallbackIK(n=len(self.joint_names))

		# example static target pose: x,y,z,qx,qy,qz,qw
		self.target_pose = [0.4, 0.2, 0.3, 0, 0, 0, 1]

		self.timer = self.create_timer(0.2, self._publish_joint_state)

	def _publish_joint_state(self):
		try:
			result = self.solver.solve(self.target_pose, base_frame="base_link", ee_frame="ee_link")
		except Exception as e:
			self.get_logger().warning("Solver exception: %s", str(e))
			result = [0.0] * len(self.joint_names)

		if len(result) < len(self.joint_names):
			result = list(result) + [0.0] * (len(self.joint_names) - len(result))

		msg = JointState()
		msg.header.stamp = self.get_clock().now().to_msg()
		msg.name = self.joint_names
		msg.position = list(result[: len(self.joint_names)])

		self.pub.publish(msg)


def main():
	rclpy.init()
	node = RangedIKNode()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		node.destroy_node()
		rclpy.shutdown()


if __name__ == "__main__":
	main()

