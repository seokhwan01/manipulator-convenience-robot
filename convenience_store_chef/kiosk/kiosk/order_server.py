import rclpy
from rclpy.node import Node
from convenience_store_chef_msgs.srv import OrderService

class OrderServer(Node):
    def __init__(self):
        super().__init__('order_server')
        self.srv = self.create_service(OrderService, '/order_service', self.handle_order)
        self.get_logger().info(f'서비스 서버 생성 완료')

    def handle_order(self, request, response):
        self.get_logger().info(f'📦 주문 항목: {request.items}')
        self.get_logger().info(f'📦 수량: {request.quantities}')
        response.success = True
        return response

def main():
    rclpy.init()
    node = OrderServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
