import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import depthai as dai
import cv2

class RGBPublisher(Node):
    def __init__(self):
        super().__init__('rgb_publisher')

        # Parâmetro configurável via launch
        self.declare_parameter('frame_rate', 10.0)
        frame_rate = self.get_parameter('frame_rate').value
        self.get_logger().info(f'Usando frame_rate = {frame_rate} FPS')

        # Criação do publisher
        self.publisher = self.create_publisher(
            CompressedImage,
            '/oak/rgb/image_raw/compressed',
            10
        )

        # Pipeline da câmera RGB
        self.pipeline = dai.Pipeline()
        cam_rgb = self.pipeline.create(dai.node.ColorCamera)
        xout = self.pipeline.create(dai.node.XLinkOut)
        xout.setStreamName("video")

        cam_rgb.setBoardSocket(dai.CameraBoardSocket.RGB)
        cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam_rgb.setFps(frame_rate)
        cam_rgb.setInterleaved(False)
        cam_rgb.setPreviewSize(640, 480)
        cam_rgb.video.link(xout.input)

        # Dispositivo OAK-D
        self.device = dai.Device(self.pipeline)
        self.q = self.device.getOutputQueue(name="video", maxSize=4, blocking=False)

        # Timer para publicar frames de acordo com a taxa
        self.timer = self.create_timer(1.0 / frame_rate, self.publish_frame)

    def publish_frame(self):
        in_frame = self.q.get()
        frame = in_frame.getCvFrame()
        _, buffer = cv2.imencode('.jpg', frame)

        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = 'jpeg'
        msg.data = buffer.tobytes()

        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RGBPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
