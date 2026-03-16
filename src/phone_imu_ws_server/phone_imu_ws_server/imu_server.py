"""
phone_imu_ws_server  —  imu_server.py

Architecture
------------
• Main thread   : asyncio event loop (websockets + HTTP server)
• Worker thread : rclpy.spin() — services all ROS 2 middleware
• Per-connection: asyncio coroutine → puts IMU msg on a queue
• Spin thread   : drains the queue and publishes

This clean separation avoids the asyncio/rclpy event-loop conflict.
"""

import asyncio
import http.server
import json
import os
import queue
import socket
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from websockets.asyncio.server import serve as ws_serve
from websockets.exceptions import ConnectionClosed
from ament_index_python.packages import get_package_share_directory
import tf2_ros
from geometry_msgs.msg import TransformStamped


def get_local_ip():
    """Detect the local IP address assigned to the primary network interface."""
    try:
        # Standard trick to get the IP: connect to external but don't send anything
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return '127.0.0.1'


# --------------------------------------------------------------------------- #
#  HTTP server — serves phone_imu.html on port 8080                           #
# --------------------------------------------------------------------------- #
def start_http_server(web_dir: str, port: int = 8080) -> http.server.HTTPServer:
    class Handler(http.server.SimpleHTTPRequestHandler):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, directory=web_dir, **kwargs)

        def log_message(self, fmt, *args):
            pass   # silence access logs

    class ReusableTCPServer(http.server.HTTPServer):
        allow_reuse_address = True

    srv = ReusableTCPServer(('0.0.0.0', port), Handler)
    t = threading.Thread(target=srv.serve_forever, daemon=True)
    t.start()
    return srv


# --------------------------------------------------------------------------- #
#  ROS 2 node                                                                  #
# --------------------------------------------------------------------------- #
class IMUServer(Node):

    def __init__(self, msg_queue: queue.Queue):
        super().__init__('phone_imu_server')

        self._queue = msg_queue
        # [-1, 0...] means "covariance unknown" in sensor_msgs/Imu
        self._unknown_cov = [-1.0] + [0.0] * 8

        self.publisher = self.create_publisher(Imu, '/phone_imu', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.last_update = self.get_clock().now()

        # Stats
        self._rx_count = 0
        self._tx_count = 0

        # Timer: drains the inter-thread queue and publishes
        self.create_timer(0.02, self._publish_queued)   # 50 Hz drain
        self.create_timer(1.0, self._log_stats)        # 1 Hz stats

        # HTTP server ── find installed web/ directory
        web_dir = os.path.join(
            get_package_share_directory('phone_imu_ws_server'), 'web')
        start_http_server(web_dir, port=8080)

        local_ip = get_local_ip()
        self.get_logger().info(
            '\n'
            '  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n'
            f'  📱 Cyber-Dashboard: http://{local_ip}:8080\n'
            f'  🔌 WebSocket Host: {local_ip}:8765\n'
            '  📡 ROS 2 Topic:    /phone_imu\n'
            '  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━')

    def _log_stats(self):
        if self._rx_count > 0:
            self.get_logger().info(f'Stats: Received {self._rx_count} msgs, Published {self._tx_count} msgs (last 1s)')
            self._rx_count = 0
            self._tx_count = 0

    def incr_rx(self):
        self._rx_count += 1

    def _publish_queued(self):
        """Drain the thread-safe queue and publish all pending IMU messages."""
        now = self.get_clock().now()
        dt = (now - self.last_update).nanoseconds / 1e9
        self.last_update = now

        while not self._queue.empty():
            try:
                data = self._queue.get_nowait()
            except queue.Empty:
                break

            imu = Imu()
            imu.header.stamp    = now.to_msg()
            imu.header.frame_id = 'phone_imu_link'

            # 1. Orientation (Quaternion)
            imu.orientation.x = float(data.get('qx', 0.0))
            imu.orientation.y = float(data.get('qy', 0.0))
            imu.orientation.z = float(data.get('qz', 0.0))
            imu.orientation.w = float(data.get('qw', 1.0))
            imu.orientation_covariance = self._unknown_cov

            # 2. Angular Velocity (rad/s)
            imu.angular_velocity.x = float(data.get('gx', 0.0))
            imu.angular_velocity.y = float(data.get('gy', 0.0))
            imu.angular_velocity.z = float(data.get('gz', 0.0))
            imu.angular_velocity_covariance = [0.0] * 9

            # 3. Linear Acceleration (m/s^2)
            ax = float(data.get('ax', 0.0))
            ay = float(data.get('ay', 0.0))
            az = float(data.get('az', 0.0))
            imu.linear_acceleration.x = ax
            imu.linear_acceleration.y = ay
            imu.linear_acceleration.z = az
            imu.linear_acceleration_covariance = [0.0] * 9

            # Odometry integration removed (as requested: only rotation/tilt shown)

            # 5. Broadcast Transform (odom -> base_link)
            t = TransformStamped()
            t.header.stamp = now.to_msg()
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_link'
            
            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.0
            
            t.transform.rotation.x = imu.orientation.x
            t.transform.rotation.y = imu.orientation.y
            t.transform.rotation.z = imu.orientation.z
            t.transform.rotation.w = imu.orientation.w
            
            self.tf_broadcaster.sendTransform(t)
            self.publisher.publish(imu)
            self._tx_count += 1


# --------------------------------------------------------------------------- #
#  WebSocket server (asyncio)                                                  #
# --------------------------------------------------------------------------- #
async def ws_handler(websocket, msg_queue: queue.Queue, node: IMUServer):
    """One coroutine per connected phone — parses JSON, enqueues data."""
    addr = websocket.remote_address
    logger = node.get_logger()
    logger.info(f'Phone connected: {addr}')
    try:
        async for raw in websocket:
            try:
                data = json.loads(raw)
                msg_queue.put_nowait(data)
                node.incr_rx()
            except (json.JSONDecodeError, queue.Full):
                pass
    except ConnectionClosed:
        pass
    finally:
        logger.info(f'Phone disconnected: {addr}')


async def ws_server(msg_queue: queue.Queue, node: IMUServer):
    async with ws_serve(
        lambda ws: ws_handler(ws, msg_queue, node),
        '0.0.0.0', 8765
    ):
        node.get_logger().info('WebSocket server listening on port 8765')
        await asyncio.get_running_loop().create_future()  # run forever


# --------------------------------------------------------------------------- #
#  Entry point                                                                 #
# --------------------------------------------------------------------------- #
def main():
    rclpy.init()

    msg_queue: queue.Queue = queue.Queue(maxsize=500)
    node = IMUServer(msg_queue)

    # rclpy spins in a background thread
    spin_thread = threading.Thread(
        target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        # asyncio handles WebSocket connections in the main thread
        asyncio.run(ws_server(msg_queue, node))
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down …')
    finally:
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)


if __name__ == '__main__':
    main()
