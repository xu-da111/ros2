#!/usr/bin/env python3
"""
多无人机协同分区覆盖搜索节点.

给定矩形搜索区域和无人机数量, 把区域沿 y 轴均分, 每架无人机在自己的
条带内走牛耕式(Boustrophedon)往复航线, 互不重叠, 适合搜救/巡查。

与 uav_formation_flight(几何编队)和 ap_cpp_overfomo(单机主动感知)的区别:
本模块是多机静态分区覆盖, 不保持几何队形, 也不做在线重规划。

话题:
  发布: /drone{i}/setpoint_position  geometry_msgs/PointStamped
  发布: /coverage_report             std_msgs/String
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String


class MultiCoverageNode(Node):
    def __init__(self):
        super().__init__("uav_multi_area_coverage")

        self.declare_parameter("area_x_min", -10.0)
        self.declare_parameter("area_x_max", 10.0)
        self.declare_parameter("area_y_min", -10.0)
        self.declare_parameter("area_y_max", 10.0)
        self.declare_parameter("flight_height", 5.0)
        self.declare_parameter("strip_spacing", 2.0)  # 牛耕相邻航线间距 m
        self.declare_parameter("num_drones", 2)
        self.declare_parameter("waypoint_interval_s", 0.3)

        self.x_min = float(self.get_parameter("area_x_min").value)
        self.x_max = float(self.get_parameter("area_x_max").value)
        self.y_min = float(self.get_parameter("area_y_min").value)
        self.y_max = float(self.get_parameter("area_y_max").value)
        self.h = float(self.get_parameter("flight_height").value)
        self.spacing = float(self.get_parameter("strip_spacing").value)
        self.N = int(self.get_parameter("num_drones").value)
        self.dt = float(self.get_parameter("waypoint_interval_s").value)

        # 每架机的条带范围 (沿 x 轴切分)
        self.strips = self._split_area()
        # 每架机的航点队列
        self.routes = [self._build_strip_route(s) for s in self.strips]
        self.indices = [0] * self.N

        self.pubs = [
            self.create_publisher(
                PointStamped, f"/drone{i+1}/setpoint_position", 10)
            for i in range(self.N)
        ]
        self.report_pub = self.create_publisher(
            String, "/coverage_report", 10)

        self.timer = self.create_timer(self.dt, self._step)
        self.get_logger().info(
            f"多机覆盖启动: {self.N} 架机, 区域 "
            f"[{self.x_min},{self.x_max}]x[{self.y_min},{self.y_max}], "
            f"条带间距 {self.spacing}m")

    def _split_area(self):
        """把区域沿 x 轴均分成 N 个条带."""
        width = self.x_max - self.x_min
        strip_w = width / self.N
        return [
            (self.x_min + i * strip_w,
             self.x_min + (i + 1) * strip_w,
             self.y_min, self.y_max)
            for i in range(self.N)
        ]

    def _build_strip_route(self, strip):
        """在单个条带内生成牛耕式往复航点."""
        sx, ex, sy, ey = strip
        route = []
        y = sy
        direction = 1  # 1: 飞到 ex, -1: 飞到 sx
        while y <= ey + 1e-6:
            if direction == 1:
                route.append((ex, y, self.h))
                route.append((sx, y, self.h))
            else:
                route.append((sx, y, self.h))
                route.append((ex, y, self.h))
            direction *= -1
            y += self.spacing
        return route

    def _step(self):
        all_done = True
        for i in range(self.N):
            if self.indices[i] < len(self.routes[i]):
                wp = self.routes[i][self.indices[i]]
                msg = PointStamped()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "map"
                msg.point.x, msg.point.y, msg.point.z = wp
                self.pubs[i].publish(msg)
                self.indices[i] += 1
                all_done = False
            else:
                # 完成后悬停在当前点
                if self.routes[i]:
                    last = self.routes[i][-1]
                    msg = PointStamped()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = "map"
                    msg.point.x, msg.point.y, msg.point.z = last
                    self.pubs[i].publish(msg)

        if all_done:
            self.report_pub.publish(String(data="ALL_DONE"))
            self.get_logger().info("所有无人机完成覆盖")
        else:
            progress = [
                f"drone{i+1}:{self.indices[i]}/{len(self.routes[i])}"
                for i in range(self.N)
            ]
            self.report_pub.publish(String(data=",".join(progress)))


def main(args=None):
    rclpy.init(args=args)
    node = MultiCoverageNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
