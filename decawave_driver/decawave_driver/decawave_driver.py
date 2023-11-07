#!/usr/bin/env python
import rclpy

import time

import serial
import struct

from decawave_msgs.msg import Tag, Anchor, AnchorArray
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from rclpy.node import Node
from rclpy.duration import Duration


class DecawaveDriver(Node):
    """docstring for DecawaveDriver"""

    def __init__(self):
        super().__init__("decawave_driver")
        # Getting Serial Parameters
        self.handle_parameters()
        self.serial_port_ = self.get_parameter("serial_port").get_parameter_value().string_value
        self.baudrate_ = self.get_parameter("baudrate").get_parameter_value().integer_value
        self.tf_publisher_ = self.get_parameter("enable_tf").get_parameter_value().bool_value
        self.tf_reference_ = self.get_parameter("tf_reference").get_parameter_value().string_value
        self.tag_name_ = self.get_parameter("tag_name").get_parameter_value().string_value
        self.rate_ = self.get_parameter("rate").get_parameter_value().integer_value
        self.serial_timeout_param = (
            self.get_parameter("serial_timeout").get_parameter_value().double_value
        )
        self.serial_timeout = Duration(seconds=self.serial_timeout_param)
        # Initiate Serial
        self.ser = serial.Serial(self.serial_port_, self.baudrate_, timeout=0.1)
        self.get_logger().info(f"\33[96mConnected to {self.ser.portstr} at {self.baudrate_}\33[0m")
        self.get_uart_mode()
        self.get_uart_mode()

        self.get_logger().info("\33[96mInitiating Driver...\33[0m")
        self.get_tag_status()
        self.get_tag_version()
        self.anchors = AnchorArray()
        self.anchors.anchors = []
        self.tag = Tag()

        self.get_logger().info("\33[96mCreating publishers...\33[0m")

        self.tag_pub_ = self.create_publisher(Tag, "tag_pose", 1)
        self.anchors_pub_ = self.create_publisher(AnchorArray, "tag_status", 1)

        self.br = TransformBroadcaster(self)
        self.static_br = StaticTransformBroadcaster(self)

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("\33[96mSpinning...\33[0m")

    def handle_parameters(self):
        self.declare_parameter("serial_port", "/dev/ttyACM0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("enable_tf", True)
        self.declare_parameter("tf_reference", "world")
        self.declare_parameter("tag_name", "tag")
        self.declare_parameter("rate", 10)
        self.declare_parameter("serial_timeout", 0.5)

    def get_uart_mode(self):
        """Check UART Mode Used"""
        self.get_logger().info("\33[96mChecking which UART mode is the gateway...\33[0m")
        self.mode_ = "UNKNOWN"
        self.ser.write(b"\r")  # Test Mode
        time.sleep(0.1)
        while self.ser.inWaiting() == 0:
            pass
        cc = self.ser.readline()
        if cc == "\r\n" and self.ser.readline() == "dwm> ":  # SHELL MODE
            self.get_logger().info(
                "\33[96mDevice is on SHELL MODE! It must to be changed to GENERIC MODE!\33[0m"
            )
            self.mode_ = "SHELL"
        elif cc == "@\x01\x01":  # GENERIC MODE
            self.get_logger().info("\33[96mDevice is on GENERIC MODE! Ok!\33[0m")
            self.mode_ = "GENERIC"
        return self.mode_

    def switch_uart_mode(self):
        if self.mode_ == "SHELL":
            self.get_logger().info("\33[96mChanging UART mode to GENERIC MODE...\33[0m")
            self.ser.write(b"quit\r")  # Go to Generic Mode
            while self.ser.inWaiting() == 0:
                pass
            self.ser.readline()
            text = self.ser.readline().replace("\n", "")
            self.get_logger().info(f"\33[96m{text}\33[0m")
        elif self.mode_ == "UNKNOWN":
            self.get_logger().warn(
                "\33[96mUnknown Mode Detected! Please reset the device and try again!\33[0m"
            )

    def get_tag_version(self):
        self.ser.flushInput()
        self.ser.write(b"\x15\x00")  # Status
        now = self.get_clock().now()
        while self.ser.inWaiting() < 21:
            if (self.get_clock().now() - now) > self.serial_timeout:
                self.get_logger().warn("Malformed packet! Ignoring tag version.")
                self.ser.flushInput()
                return None
        version = self.ser.read(21)
        data_ = struct.unpack("<BBBBBLBBLBBL", bytearray(version))
        self.get_logger().info("\33[96m--------------------------------\33[0m")
        self.get_logger().info("\33[96mFirmware Version:0x" + format(data_[5], "04X") + "\33[0m")
        self.get_logger().info(
            "\33[96mConfiguration Version:0x" + format(data_[8], "04X") + "\33[0m"
        )
        self.get_logger().info("\33[96mHardware Version:0x" + format(data_[11], "04X") + "\33[0m")
        self.get_logger().info("\33[96m--------------------------------\33[0m")

    def get_tag_acc(self):
        # Acc is not implemented on Generic Mode
        self.ser.flushInput()
        self.ser.write(b"\x19\x33\x04")  # Status
        while self.ser.inWaiting() == 0:
            pass
        data_ = self.ser.readline()
        print("%s", data_)

    def get_tag_status(self):
        self.ser.flushInput()
        self.ser.write(b"\x32\x00")  # Status
        while self.ser.inWaiting() == 0:
            pass
        status = self.ser.readline()
        data_ = struct.unpack("<BBBBBB", bytearray(status))
        if data_[0] != 64 and data_[2] != 0:
            self.get_logger().warn("Get Status Failed! Packet does not match!")
            print("%s", data_)
        if data_[5] == 3:
            self.get_logger().warn(
                "\33[96mTag is CONNECTED to a UWB network and LOCATION data are READY!\33[0m"
            )
        elif data_[5] == 2:
            self.get_logger().warn(
                "Tag is CONNECTED to a UWB network but LOCATION data are NOT READY!"
            )
        elif data_[5] == 1:
            self.get_logger().warn(
                "Tag is NOT CONNECTED to a UWB network but LOCATION data are READY!"
            )
        elif data_[5] == 0:
            self.get_logger().warn(
                "Tag is NOT CONNECTED to a UWB network and LOCATION data are NOT READY!"
            )

    def get_tag_location(self):
        self.ser.flushInput()
        self.ser.write(b"\x0c\x00")
        now = self.get_clock().now()
        while self.ser.inWaiting() < 21:
            if (self.get_clock().now() - now) > self.serial_timeout:
                self.get_logger().warn("Malformed packet! Ignoring tag location.")
                self.ser.flushInput()
                return None
        data_ = self.ser.read(21)
        data_ = struct.unpack("<BBBBBlllBBBB", bytearray(data_))
        self.tag.x = float(data_[5]) / 1000.0
        self.tag.y = float(data_[6]) / 1000.0
        self.tag.z = float(data_[7]) / 1000.0
        self.tag.qf = float(data_[8]) / 100.0
        self.tag.n_anchors = int(data_[11])
        self.tag.header.frame_id = self.get_namespace() + "/" + self.tag_name_

        self.anchor_packet_size = 20  # Size of anchor packet in bytes
        now = self.get_clock().now()
        while self.ser.inWaiting() < self.anchor_packet_size * self.tag.n_anchors:
            if (self.get_clock().now() - now) > self.serial_timeout:
                self.get_logger().warn("Malformed packet! Ignoring anchors location.")
                self.ser.flushInput()
                return None
        self.anchors.anchors = []  # Clean Anchors list
        for _ in range(self.tag.n_anchors):
            data_ = self.ser.read(self.anchor_packet_size)
            data_ = struct.unpack("<HlBlllB", bytearray(data_))
            a = Anchor()
            a.header.frame_id = str(format(data_[0], "04X"))
            a.header.stamp = self.get_clock().now().to_msg()
            a.distance = float(data_[1]) / 1000.0
            a.dist_qf = float(data_[2]) / 100.0
            a.x = float(data_[3]) / 1000.0
            a.y = float(data_[4]) / 1000.0
            a.z = float(data_[5]) / 1000.0
            a.qf = float(data_[6]) / 100.0
            self.anchors.anchors.append(a)

    def publish_transform(self):
        if self.tf_publisher_ is True:
            world_to_tag_trans = TransformStamped()
            world_to_tag_trans.header.frame_id = self.get_namespace() + "/" + self.tag_name_
            world_to_tag_trans.child_frame_id = self.tf_reference_
            world_to_tag_trans.header.stamp = self.get_clock().now().to_msg()
            world_to_tag_trans.transform.translation.x = self.tag.x
            world_to_tag_trans.transform.translation.y = self.tag.y
            world_to_tag_trans.transform.translation.z = self.tag.z
            self.br.sendTransform(world_to_tag_trans)

            # TODO: @delipl RViz doesn't see many tfs published from one node
            # tfs = []
            # for anchor in self.anchors.anchors:
            #     anchor_to_world = TransformStamped()
            #     anchor_to_world.header.frame_id = anchor.header.frame_id
            #     anchor_to_world.child_frame_id = self.tf_reference_
            #     anchor_to_world.header.stamp = self.get_clock().now().to_msg()
            #     anchor_to_world.transform.translation.x = self.tag.x
            #     anchor_to_world.transform.translation.y = self.tag.y
            #     anchor_to_world.transform.translation.z = self.tag.z
            #     tfs.append(anchor_to_world)

            # self.static_br.sendTransform(tfs)

    def timer_callback(self):
        self.get_tag_location()
        self.publish_transform()
        self.tag.header.stamp = self.get_clock().now().to_msg()
        self.tag_pub_.publish(self.tag)
        self.anchors.header.stamp = self.get_clock().now().to_msg()
        self.anchors_pub_.publish(self.anchors)


def main(args=None):
    rclpy.init(args=args)
    decawave_driver = DecawaveDriver()
    rclpy.spin(decawave_driver)
    decawave_driver.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
