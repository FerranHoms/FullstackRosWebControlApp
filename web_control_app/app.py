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
        msg.linear.x = float(solenoid_number)
        msg.linear.y = float(duration_seconds * 1000)
        msg.angular.x = 1.0 if start else -1.0 if stop else 0.0
        self.publisher.publish(msg)
        self.get_logger().info(
            f"Sent Dispenser {solenoid_number} (solenoid): Duration {duration_seconds}s, Start: {start}, Stop: {stop}"
        )

    def send_pump_command(self, pump_number, duration_seconds=0, start=False, stop=False):
        msg = Twist()
        msg.linear.z = float(pump_number)
        msg.angular.y = float(duration_seconds * 1000)
        msg.angular.z = 1.0 if start else -1.0 if stop else 0.0
        self.publisher.publish(msg)
        self.get_logger().info(
            f"Sent Dispenser {pump_number+4} (pump): Duration {duration_seconds}s, Start: {start}, Stop: {stop}"
        )

rclpy.init()
node = ControlNode()

@app.route('/coffee', methods=['POST'])
def coffee_control():
    data = request.json
    dispenser_number = data.get('dispenser_number')
    if dispenser_number is None or not (1 <= dispenser_number <= 8):
        return jsonify({'status': 'error', 'message': 'Invalid dispenser number (must be 1-8)'}), 400

    duration_seconds = data.get('duration_seconds', 0)
    start = data.get('start', False)
    stop = data.get('stop', False)

    if dispenser_number <= 4:
        # For dispensers 1-4, use solenoid command
        node.send_solenoid_command(dispenser_number, duration_seconds, start, stop)
    else:
        # For dispensers 5-8, map to pump relays (subtract 4)
        node.send_pump_command(dispenser_number - 4, duration_seconds, start, stop)
    return jsonify({'status': 'success'})

def main():
    app.run(host='0.0.0.0', port=5000)

if __name__ == '__main__':
    main()
