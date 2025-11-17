import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class TFHelper(Node):

    def __init__(self):
        super().__init__('tf_helper')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Optional: check TF every second just to show it's alive
        # self.timer = self.create_timer(1.0, self.print_available_frames)


    def lookup_transform(self, source: str, target: str):
        """Look up transform from source → target frame."""
        try:
            transform = self.tf_buffer.lookup_transform(
                target,
                source,
                rclpy.time.Time()      # time=0 → most recent transform
            )
            print(1929)
            print( f"TF lookup successful: {source} → {target}", transform )
            return transform
        except TransformException as ex:
            self.get_logger().warn(f"TF lookup failed: {source} → {target}: {ex}")
            return None

    def get_transform(self, source: str, target: str):
        # now TF buffer is up to date
        t = self.lookup_transform(source, target)

        # It is important to check if tranform is present or not, as often in the beginning the listener has not yet received any data
        if t is not None:
            trans = t.transform.translation
            rot = t.transform.rotation

            print("Translation:", trans.x, trans.y, trans.z)
            print("Rotation (quat):", rot.x, rot.y, rot.z, rot.w)
        else:
            print("No transform yet!")


    def print_available_frames(self):
        frames = self.tf_buffer.all_frames_as_string()
        frames_yaml = self.tf_buffer.all_frames_as_yaml()
        # self.get_logger().info(f"\nCurrent TF frames:\n{frames}")
        print(f"\nCurrent TF frames (YAML):\n{frames_yaml}")


def main():
    rclpy.init()
    node = TFHelper()

    # Example usage:
    while rclpy.ok():
        rclpy.spin_once(node)
        node.get_transform("right_delto_base_link", "table")

    rclpy.shutdown()
if __name__ == '__main__':
    main()
