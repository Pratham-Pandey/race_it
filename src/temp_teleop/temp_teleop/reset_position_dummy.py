import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty


class ResetSim(Node):
    def __init__(self):
        super().__init__("minimum_reset_client")
        self.client = self.create_client(Empty, "/reset_world")
        while not self.client.wait_for_service():
            self.get_logger().info("Service not available, waiting again...")
        self.req = Empty.Request()

    def send_request(self):
        self.future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
