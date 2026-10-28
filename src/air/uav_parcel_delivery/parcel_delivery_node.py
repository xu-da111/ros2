#!/usr/bin/env python3
"""
无人机空投投递节点.

依次飞抵投递点列表上空, 位置锁定后触发放物机构, 再飞向下一点,
全部完成后返航。

话题:
  订阅: /drone/position          geometry_msgs/PointStamped
  发布: /drone/setpoint_position geometry_msgs/PointStamped
  发布: /drone/drop_command      std_msgs/Bool (True=投放)
  发布: /delivery_status          std_msgs/String
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool, String


class ParcelDeliveryNode(Node):
    def __init__(self):
        super().__init__("uav_parcel_delivery")

        # 投递点 "x,y,z;x,y,z"  z 为投放高度
        self.declare_parameter(
            "drop_points",
            "5.0,5.0,3.0; -5.0,-5.0,3.0; 0.0,8.0,3.0")
        self.declare_parameter("position_tol", 0.5)  # 位置锁定容差 m
        self.declare_parameter("hover_before_drop_s", 1.0)  # 悬停稳定时间
        self.declare_parameter("home_x", 0.0)
        self.declare_parameter("home_y", 0.0)
        self.declare_parameter("home_z", 0.0)
        self.declare_parameter("timer_period_s", 0.2)

        self.drop_points = []
        for item in self.get_parameter("drop_points").value.split(";"):
            try:
                x, y, z = [float(v) for v in item.split(",")]
                self.drop_points.append((x, y, z))
            except ValueError:
                pass

        self.tol = float(self.get_parameter("position_tol").value)
        self.hover_time = float(self.get_parameter("hover_before_drop_s").value)
        self.home = (
            float(self.get_parameter("home_x").value),
            float(self.get_parameter("home_y").value),
            float(self.get_parameter("home_z").value),
        )

        self.pos = (0.0, 0.0, 0.0)
        self.current_idx = 0
        self.hover_accum = 0.0
        self.dropped = False
        self.state = "FLYING_TO_POINT"

        self.pos_sub = self.create_subscription(
            PointStamped, "/drone/position", self._pos_cb, 10)
        self.target_pub = self.create_publisher(
            PointStamped, "/drone/setpoint_position", 10)
        self.drop_pub = self.create_publisher(
            Bool, "/drone/drop_command", 10)
        self.status_pub = self.create_publisher(
            String, "/delivery_status", 10)

        self.timer = self.create_timer(
            float(self.get_parameter("timer_period_s").value), self._loop)
        self.get_logger().info(
            f"空投投递启动, 共 {len(self.drop_points)} 个投递点")

    def _pos_cb(self, msg: PointStamped):
        self.pos = (msg.point.x, msg.point.y, msg.point.z)

    def _dist(self, a, b):
        return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))

    def _publish(self, p):
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.point.x, msg.point.y, msg.point.z = p
        self.target_pub.publish(msg)

    def _loop(self):
        if self.current_idx < len(self.drop_points):
            target = self.drop_points[self.current_idx]
            self._publish(target)

            if self._dist(self.pos, target) < self.tol:
                self.hover_accum += 0.2
                if self.hover_accum >= self.hover_time and not self.dropped:
                    self.drop_pub.publish(Bool(data=True))
                    self.get_logger().info(
                        f"已在投递点 {target} 投放物资 "
                        f"({self.current_idx+1}/{len(self.drop_points)})")
                    self.dropped = True
                    self.hover_accum = 0.0
            else:
                self.hover_accum = 0.0
                self.dropped = False

            if self.dropped and self.hover_accum >= 0.5:
                self.current_idx += 1
                self.dropped = False
                self.hover_accum = 0.0
        else:
            self._publish(self.home)
            self.state = "RETURNING_HOME"

        self.status_pub.publish(String(data=self.state))


def main(args=None):
    rclpy.init(args=args)
    node = ParcelDeliveryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
