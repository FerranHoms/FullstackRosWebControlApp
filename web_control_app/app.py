from flask import Flask, request, jsonify
from flask_cors import CORS
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

app = Flask(__name__)
CORS(app)

class ControlNode(Node):
    def __init__(self):
        super().__init__('web_control_node')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

    def send_solenoid_command(self, solenoid_number, duration_seconds=0, start=False, stop=False):
        msg = Twist()
        msg.linear.x = float(solenoid_number)  # Solenoid index (1-4)
        msg.linear.y = float(duration_seconds * 1000)  # Duration in milliseconds
        msg.angular.x = 1.0 if start else -1.0 if stop else 0.0  # Start or stop flag
        self.publisher.publish(msg)
        self.get_logger().info(f'Sent Solenoid {solenoid_number}: Duration {duration_seconds}s, Start: {start}, Stop: {stop}')

    def send_pump_command(self, pump_number, duration_seconds=0, start=False, stop=False):
        msg = Twist()
        msg.linear.z = float(pump_number)  # Pump index (1-4)
        msg.angular.y = float(duration_seconds * 1000)  # Duration in milliseconds
        msg.angular.z = 1.0 if start else -1.0 if stop else 0.0  # Start or stop flag
        self.publisher.publish(msg)
        self.get_logger().info(f'Sent Pump {pump_number}: Duration {duration_seconds}s, Start: {start}, Stop: {stop}')

rclpy.init()
node = ControlNode()

@app.route('/solenoid', methods=['POST'])
def solenoid_control():
    data = request.json
    solenoid_number = data.get('solenoid_number')
    duration_seconds = data.get('duration_seconds', 0)
    start = data.get('start', False)
    stop = data.get('stop', False)
    node.send_solenoid_command(solenoid_number, duration_seconds, start, stop)
    return jsonify({'status': 'success'})

@app.route('/pump', methods=['POST'])
def pump_control():
    data = request.json
    pump_number = data.get('pump_number')
    duration_seconds = data.get('duration_seconds', 0)
    start = data.get('start', False)
    stop = data.get('stop', False)
    node.send_pump_command(pump_number, duration_seconds, start, stop)
    return jsonify({'status': 'success'})

def main():
    app.run(host='0.0.0.0', port=5000)

if __name__ == '__main__':
    main()