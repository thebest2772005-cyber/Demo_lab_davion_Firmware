#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class LaserScanFilter180(Node):
    def __init__(self):
        super().__init__('laser_scan_filter_180')
        
        # Subcribe topic 360 độ mặc định của LiDAR
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        # Publish topic 180 độ đã qua bộ lọc
        self.publisher = self.create_publisher(LaserScan, '/scan_filtered', 10)
        self.get_logger().info('Đã bật bộ lọc 180 độ (đã thêm tính năng đảo chiều)!')

    def scan_callback(self, msg: LaserScan):
        filtered_msg = LaserScan()
        filtered_msg.header = msg.header
        filtered_msg.time_increment = msg.time_increment
        filtered_msg.scan_time = msg.scan_time
        filtered_msg.range_min = msg.range_min
        filtered_msg.range_max = msg.range_max
        filtered_msg.angle_increment = msg.angle_increment

        # Lấy 180 độ phía sau: +90 đến +270 độ. Với scan đầu vào -180 đến
        # +180 độ, vùng này nằm ở cuối và đầu mảng nên cần ghép hai đoạn.
        rear_start = math.pi / 2.0       # +90 độ

        upper_start_idx = math.ceil(
            (rear_start - msg.angle_min) / msg.angle_increment
        )
        lower_end_idx = math.floor(
            (-math.pi / 2.0 - msg.angle_min) / msg.angle_increment
        )
        upper_start_idx = max(0, min(len(msg.ranges), upper_start_idx))
        lower_end_idx = max(-1, min(len(msg.ranges) - 1, lower_end_idx))

        # Thứ tự góc sau khi ghép vẫn tăng liên tục: +90 -> +270 độ.
        sliced_ranges = list(msg.ranges[upper_start_idx:]) + list(msg.ranges[:lower_end_idx + 1])
        if msg.intensities:
            sliced_intensities = (
                list(msg.intensities[upper_start_idx:])
                + list(msg.intensities[:lower_end_idx + 1])
            )
        else:
            sliced_intensities = []

        start_angle = msg.angle_min + upper_start_idx * msg.angle_increment
        end_angle = start_angle + (len(sliced_ranges) - 1) * msg.angle_increment

        # ==========================================
        # CẤU HÌNH ĐẢO CHIỀU (Đổi True/False để sửa lỗi)
        # ==========================================
        
        # 1. Sửa lỗi ngược Trái - Phải (Vật cản bên trái hiện sang phải)
        flip_left_right = False
        
        # 2. Dữ liệu đã được cắt đúng vùng phía sau, không xoay thêm.
        flip_front_back = False
        
        # Xử lý đảo mảng (Trái - Phải)
        if flip_left_right:
            sliced_ranges = sliced_ranges[::-1]
            if sliced_intensities:
                sliced_intensities = sliced_intensities[::-1]
                
        # Xử lý xoay góc (Trước - Sau)
        if flip_front_back:
            # Cộng thêm 180 độ (Pi radian) để đưa dữ liệu ra phía trước
            filtered_msg.angle_min = start_angle + math.pi
            filtered_msg.angle_max = end_angle + math.pi
        else:
            filtered_msg.angle_min = start_angle
            filtered_msg.angle_max = end_angle

        # ==========================================

        # Gán dữ liệu đã xử lý vào message
        filtered_msg.ranges = sliced_ranges
        filtered_msg.intensities = sliced_intensities

        # Phát dữ liệu đã lọc
        self.publisher.publish(filtered_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LaserScanFilter180()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
