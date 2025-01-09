import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from flask import Flask, request, jsonify
import threading
from geometry_msgs.msg import Polygon, Point32
from flask_cors import CORS

class HttpRosNode(Node):

    def __init__(self):
        super().__init__('httpRosNode')
        self.GUI_command_sub = self.create_subscription(String, 'GUI_command', self.GUI_command_callback, 10)

        self.publisher_ = self.create_publisher(Polygon, 'SC_Waypoints', 10)
        timer_period = 2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        # Create an event that will be used to stop the Flask server
        self.thread_stop_event = threading.Event()

        # Flask HTTP Server in a separate thread
        self.flask_thread = threading.Thread(target=self.run_flask_server, daemon=True)
        self.flask_thread.start()

    def timer_callback(self):
        # msg = String()
        # msg.data = 'Operational' 
        # self.publisher_.publish(msg)
        self.get_logger().info('Operational')
        # self.i += 1

    def GUI_command_callback(self, msg):
        """
        Terminate this node when the GUI sends a terminate command (closing the GUI).
        """
        self.GUI_command = msg

        if msg.data == "terminate/kill all":
            # Signal the Flask thread to stop
            self.thread_stop_event.set()
            self.flask_thread.join()  # Wait for the thread to finish
            self.destroy_node()
            sys.exit()

    def run_flask_server(self):
        # Create Flask app to handle HTTP requests
        app = Flask(__name__)
        CORS(app)

        @app.route('/api/drones', methods=['POST'])
        def handle_post_drones():
            # Retrieve JSON data from the POST request
            data = request.get_json()
            self.get_logger().info(f"Received data: {data}")            
            if data:
                # Process the data, for example, publish it to a ROS topic

                dictionary = data[0]

                msg = Polygon()
                point = Point32()
                point.x, point.y, point.z = float(dictionary['x']), float(dictionary['y']), float(dictionary['z'])
                msg.points.append(point)

                self.publisher_.publish(msg)
                # self.get_logger().info(f"Published: {msg.data}")

                return jsonify({"status": "success", "message": "Data received"}), 200
            else:
                return jsonify({"status": "error", "message": "No data received"}), 400

        # Run the Flask app (this will run until the thread_stop_event is set)
        while not self.thread_stop_event.is_set():
            try:
                app.run(host='0.0.0.0', port=3000, threaded=True, use_reloader=False)
            except Exception as e:
                self.get_logger().info(f"Flask server encountered an error: {str(e)}")
                break  # If an error occurs, exit the loop to stop the server

def main(args=None):
    rclpy.init(args=args)
    httpRosNode = HttpRosNode()
    rclpy.spin(httpRosNode)
    httpRosNode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
