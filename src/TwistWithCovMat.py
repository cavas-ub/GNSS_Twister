# Author: DeeKay Goswami

import rclpy
from geometry_msgs.msg import TwistWithCovarianceStamped
from novatel_oem7_msgs.msg import INSPVA, INSSTDEV

class TwisterNode:
    def __init__(self):
        self.node = rclpy.create_node("twister")

        self.inspva_sub = self.node.create_subscription(INSPVA, "/novatel/oem7/inspva", self.inspva_callback, 50)
        self.insstdev_sub = self.node.create_subscription(INSSTDEV, "/novatel/oem7/insstdev", self.insstdev_callback, 1)

        self.twist_pub = self.node.create_publisher(TwistWithCovarianceStamped, "/novatel/oem7/twistwithcovstamped", 50)

        self.standard_deviations = {
            "linear_velocity_X": 0.0,
            "linear_velocity_Y": 0.0,
            "linear_velocity_Z": 0.0,
            "angular_velocity_X": 0.0,
            "angular_velocity_Y": 0.0,
            "angular_velocity_Z": 0.0
        }

    def inspva_callback(self, msg):
        linear_velocity_x = msg.east_velocity
        linear_velocity_y = msg.north_velocity
        linear_velocity_z = msg.up_velocity
        angular_velocity_x = msg.roll
        angular_velocity_y = msg.pitch
        angular_velocity_z = msg.azimuth

        twist_msg = TwistWithCovarianceStamped()
        twist_msg.header.stamp = self.node.get_clock().now().to_msg()
        twist_msg.header.frame_id = "base_link"
        twist_msg.twist.twist.linear.x = linear_velocity_x
        twist_msg.twist.twist.linear.y = linear_velocity_y
        twist_msg.twist.twist.linear.z = linear_velocity_z
        twist_msg.twist.twist.angular.x = angular_velocity_x
        twist_msg.twist.twist.angular.y = angular_velocity_y
        twist_msg.twist.twist.angular.z = angular_velocity_z

        covariance_matrix = [0.0] * 36
        covariance_matrix[0] = self.standard_deviations["linear_velocity_X"] ** 2
        covariance_matrix[7] = self.standard_deviations["linear_velocity_Y"] ** 2
        covariance_matrix[14] = self.standard_deviations["linear_velocity_Z"] ** 2
        covariance_matrix[21] = self.standard_deviations["angular_velocity_X"] ** 2
        covariance_matrix[28] = self.standard_deviations["angular_velocity_Y"] ** 2
        covariance_matrix[35] = self.standard_deviations["angular_velocity_Z"] ** 2

        twist_msg.twist.covariance = covariance_matrix

        self.twist_pub.publish(twist_msg)

    def insstdev_callback(self, msg):
        self.standard_deviations["linear_velocity_X"] = msg.east_velocity_stdev
        self.standard_deviations["linear_velocity_Y"] = msg.north_velocity_stdev
        self.standard_deviations["linear_velocity_Z"] = msg.up_velocity_stdev
        self.standard_deviations["angular_velocity_X"] = msg.roll_stdev
        self.standard_deviations["angular_velocity_Y"] = msg.pitch_stdev
        self.standard_deviations["angular_velocity_Z"] = msg.azimuth_stdev

def main(args=None):
    rclpy.init(args=args)

    try:
        Twister_Node = TwisterNode()
        rclpy.spin(Twister_Node.node)
    except KeyboardInterrupt:
        pass
    finally:
        Twister_Node.node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()