import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped


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


    def print_available_frames(self):
        """
        Look for all available TF frames and print them into terminal

        Args:
            None
        Returns:
            None
        """
        frames = self.tf_buffer.all_frames_as_string()
        frames_yaml = self.tf_buffer.all_frames_as_yaml()
        # self.get_logger().info(f"\nCurrent TF frames:\n{frames}")
        print(f"\nCurrent TF frames (YAML):\n{frames_yaml}")


def main():
    rclpy.init()
    node = TFHelper()

    # Example usage:
    source = "table"
    target = "table_top"
    while rclpy.ok():
        rclpy.spin_once(node)
        t = node.lookup_transform(source, target)
        if t is None:
            print("Transform not found")
            continue
        trans = t.transform.translation
        rot = t.transform.rotation

        print(f"Transform from {source} to {target}:")
        print("translation", trans.x, ",", trans.y, ",", trans.z)
        print("rotation (quat)", rot.x, ",", rot.y, ",", rot.z, ",", rot.w)

    rclpy.shutdown()
if __name__ == '__main__':
    main()
