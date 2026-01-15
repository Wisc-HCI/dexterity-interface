import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped
import argparse


class TFHelper(Node):

    def __init__(self):
        """
        This node provides a listener to /tf topic, which gives the transforms between different frames. 

        Args:
            None

        Returns:
            None
        """
        super().__init__('tf_helper')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Optional: check TF every second just to show it's alive
        # self.timer = self.create_timer(1.0, self.print_available_frames)


    def lookup_transform(self, source: str, target: str) -> TransformStamped:
        """
        Lookup the transform from source frame -> target frame.

        Args:
            source (str): Source frame name
            target (str): Target frame name

        Returns:
            TransformStamped: The transform if found, else None
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                target,
                source,
                rclpy.time.Time()      # time=0 → most recent transform
            )

            return transform
        except TransformException as ex:
            self.get_logger().warn(f"TF lookup failed: {source} → {target}: {ex}")
            return None

def main(args=None):
    """
    This main function demonstrates how to use the TFHelper node to lookup transforms between frames.
    """

    # Parse CLI arguments
    parser = argparse.ArgumentParser(description="TFHelper CLI")

    parser.add_argument('--source', type=str, default='table',
                        help='Source frame name')
    parser.add_argument('--target', type=str, default='table_top',
                        help='Target frame name')
    args_parsed = parser.parse_args(args)

    rclpy.init(args=args)
    node = TFHelper()

    source = args_parsed.source
    target = args_parsed.target

    while rclpy.ok():
        rclpy.spin_once(node)
        t = node.lookup_transform(source, target)
        if t is None:
            print(f"Transform not found: {source} → {target}")
            continue

        trans = t.transform.translation
        rot = t.transform.rotation

        print(f"Transform from {source} to {target}:")
        print("translation:", trans.x, trans.y, trans.z)
        print("rotation (quat):", rot.x, rot.y, rot.z, rot.w)

    rclpy.shutdown()


if __name__ == '__main__':
    main()