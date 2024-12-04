import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from flask import Flask, request, jsonify
import threading

class HttpRosNode(Node):

    def __init__(self):
        super().__init__('httpRosNode')
        self.GUI_command_sub = self.create_subscription(String, 'GUI_command', self.GUI_command_callback, 10)

        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        # Flask HTTP Server in a separate thread
        self.flask_thread = threading.Thread(target=self.run_flask_server)
        self.flask_thread.start()

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

    def GUI_command_callback(self, msg):
        """
        Terminate this node when the GUI sends a terminate command (closing the GUI).
        """
        self.GUI_command = msg

        if msg.data == "terminate/kill all":
            self.destroy_node()
            sys.exit()

    def run_flask_server(self):
        # Create Flask app to handle HTTP requests
        app = Flask(__name__)

        @app.route('/api/drones', methods=['POST'])
        def handle_post_drones():
            # Retrieve JSON data from the POST request
            data = request.get_json()
            if data:
                # Log or handle the data as needed
                self.get_logger().info(f"Received data: {data}")

                # Process the data, for example, publish it to a ROS topic
                msg = String()
                msg.data = str(data)  # Convert data to string (or use specific data)
                self.publisher_.publish(msg)
                self.get_logger().info(f"Published: {msg.data}")

                return jsonify({"status": "success", "message": "Data received"}), 200
            else:
                return jsonify({"status": "error", "message": "No data received"}), 400

        # Run the Flask app
        app.run(host='127.0.0.1', port=5000)

def main(args=None):
    rclpy.init(args=args)
    httpRosNode = HttpRosNode()
    rclpy.spin(httpRosNode)
    httpRosNode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
