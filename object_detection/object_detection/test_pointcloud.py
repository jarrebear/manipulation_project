
import rclpy
from rclpy.node import Node

import numpy as np
import tf2_ros

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from tf2_ros import TransformException


class PointCloudInspector(Node):

    def __init__(self):
        super().__init__('pointcloud_inspector')

        self.subscription = self.create_subscription(
            PointCloud2,
            '/wrist_rgbd_depth_sensor/points',
            self.callback,
            10
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer, self
        )

        self.get_logger().info('PointCloud inspector started!')

    def callback(self, msg):

        try:
            # Get transform into base_link
            transform = self.tf_buffer.lookup_transform(
                'base_link',
                msg.header.frame_id,
                rclpy.time.Time.from_msg(msg.header.stamp),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )

            t = transform.transform.translation
            q = transform.transform.rotation

            translation = np.array([t.x, t.y, t.z])

            # Quaternion to rotation matrix
            x, y, z, w = q.x, q.y, q.z, q.w

            R = np.array([
                [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
                [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
                [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)]
            ])

            points = point_cloud2.read_points(
                msg,
                field_names=('x', 'y', 'z'),
                skip_nans=True
            )

            if points.size == 0:
                return

            # Convert structured array into a regular Nx3 NumPy array
            points = np.column_stack((
                points['x'],
                points['y'],
                points['z']
            )).astype(np.float64)

            # Transform points into base_link
            points = points @ R.T + translation

            # Remove NaN and Inf values
            valid_mask = np.isfinite(points).all(axis=1)
            points = points[valid_mask]

            if len(points) == 0:
                self.get_logger().warning("No valid points found!")
                return

            # Extract coordinates
            x_values = points[:, 0]
            y_values = points[:, 1]
            z_values = points[:, 2]

            self.get_logger().info(
                f"\n--- POINT CLOUD STATISTICS ---\n"
                f"Points: {len(points)}\n"
                f"X: {np.min(x_values):.3f} to {np.max(x_values):.3f}\n"
                f"Y: {np.min(y_values):.3f} to {np.max(y_values):.3f}\n"
                f"Z: {np.min(z_values):.3f} to {np.max(z_values):.3f}\n"
                f"Z percentiles:\n"
                f"  5%:  {np.percentile(z_values, 5):.3f}\n"
                f"  10%: {np.percentile(z_values, 10):.3f}\n"
                f"  25%: {np.percentile(z_values, 25):.3f}\n"
                f"  50%: {np.percentile(z_values, 50):.3f}\n"
                f"  75%: {np.percentile(z_values, 75):.3f}\n"
                f"  90%: {np.percentile(z_values, 90):.3f}\n"
                f"  95%: {np.percentile(z_values, 95):.3f}\n"
                f"------------------------------"
            )

            # Inspect height bands
            bands = [
                (0.00, 0.05),
                (0.05, 0.10),
                (0.10, 0.15),
                (0.15, 0.20),
                (0.20, 0.30),
                (0.30, 0.50),
            ]

            for low, high in bands:
                count = np.count_nonzero(
                    (z_values >= low) & (z_values < high)
                )

                self.get_logger().info(
                    f"Z [{low:.2f}, {high:.2f}): {count} points"
                )

        except TransformException as e:
            self.get_logger().warning(f'TF unavailable: {e}')


def main(args=None):

    rclpy.init(args=args)

    node = PointCloudInspector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
