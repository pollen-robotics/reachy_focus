"""ROS node controlling zoom of both cameras.

zoom has three states 'in, 'inter', 'out'
control is done by up and down arrow keys
"""

import rclpy
from rclpy.node import Node
from pynput import keyboard

from reachy_msgs.srv import SetCameraZoomLevel
from reachy_msgs.srv import Set2CamerasZoomLevel


class CameraZoomClient(Node):
    """The CameraZoomClient class allow to control zoom level through keyboard interputions"""

    def __init__(self):
        """Setup clients allowing zoom control and keyboard listener thread

        set_camera_zoom_client can control only one camera at a time
        set_2_cameras_zoom_client can controle both zoom cameras at once
        """
        super().__init__('camera_zoom_client')
        self.set_camera_zoom_client = self.create_client(SetCameraZoomLevel,
                                                         'set_camera_zoom_level')
        while not self.set_camera_zoom_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SetCameraZoomLevel.Request()

        self.set_2_cameras_zoom_client = self.create_client(Set2CamerasZoomLevel,
                                                            'set_2_cameras_zoom_level')
        while not self.set_2_cameras_zoom_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req_2_cam = Set2CamerasZoomLevel.Request()

        self.zoom = 'inter'

        self.keyboard_listener = keyboard.Listener(on_press=self.on_press)
        self.keyboard_listener.start()

    def send_request(self, name, zoom):
        """Send request through "set_camera_zoom_client" client"""
        self.req.name = name
        self.req.zoom_level = zoom
        self.future = self.set_camera_zoom_client.call_async(self.req)

    def send_request_2_cam(self, left_zoom, right_zoom):
        """Send request through "set_2_cameras_zoom_client" client"""
        self.req_2_cam.left_zoom = left_zoom
        self.req_2_cam.right_zoom = right_zoom
        self.future_2_cam = self.set_2_cameras_zoom_client.call_async(self.req_2_cam)

    def on_press(self, key):
        """Callback function call after key press event

        "up arrow" key: zoom in
        "down arrow" key: zoom out
        """
        if str(key) == "Key.up":
            if self.zoom == 'out':
                self.zoom = 'inter'
            elif self.zoom == 'inter':
                self.zoom = 'in'

        if str(key) == "Key.down":
            if self.zoom == 'in':
                self.zoom = 'inter'
            elif self.zoom == 'inter':
                self.zoom = 'out'

        self.send_request_2_cam(self.zoom, self.zoom)
        # self.send_request('left_eye',self.zoom)
        # self.test_response(self.future)
        # self.send_request('right_eye',self.zoom)

    def test_response(self, future):
        """Wait for service answer"""
        while(rclpy.ok()):
            if future.done():
                try:
                    _ = future.result()
                except Exception as e:
                    self.get_logger().info(
                        'Service call failed %r' % (e,))
                break


def main(args=None):
    """Main function

    Create and launch CameraZoomClient Node
    """
    rclpy.init(args=args)
    camera_zoom_client = CameraZoomClient()
    rclpy.spin(camera_zoom_client)


if __name__ == '__main__':
    main()
