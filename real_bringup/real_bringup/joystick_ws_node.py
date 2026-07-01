import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist   # changed from TwistStamped — hardware_bridge_node.py subscribes to plain Twist on /cmd_vel
import asyncio     #asyncio is needed for websockets, but we will run it in a separate thread
import websockets  #library to create a websocket server
import json
import threading

# MAX: 70% of rated max speed (1.2 m/s)
# MIN: kept as a joystick deadzone threshold — below this the stick input is too small to be a meaningful command
MAX_LINEAR  = 0.84   # m/s
MIN_LINEAR  = 0.10   # m/s
MAX_ANGULAR = 1.5    # rad/s
MIN_ANGULAR = 0.1    # rad/s

class JoystickWebSocketNode(Node):
    def __init__(self):
        super().__init__('joystick_ws_node')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)   # changed topic + msg type to match hardware_bridge_node.py
        self.timer = self.create_timer(0.05, self.publish_cmd)  # 20Hz
        #stores current velocity
        self.linear_x = 0.0
        self.angular_z = 0.0
        #decay factor to smoothly reduce velocity when no new commands are received not suddenly stop the robot
        self.decay = 0.65

    def clamp_velocity(self, value, max_val, min_val):

        """
        clamping: if the value is above max_val, it will be set to max_val and if
        it is below -max_val, it will be set to -max_val,
        if it is between -min_val and min_val, it will be set to 0.0, otherwise it will be unchanged
        Example with MAX_LINEAR=0.84, MIN_LINEAR=0.10:
          0.70  → 0.84   (clamped to max)
          0.30  → 0.30   (unchanged, in valid range)
          0.05  → 0.00   (below min, zeroed)
         -0.30  → -0.30  (unchanged)
         -0.80  → -0.84  (clamped to -max)
        """
        if abs(value) < min_val:
            return 0.0
        return max(-max_val, min(max_val, value))

    def publish_cmd(self):
        msg = Twist()   # changed from TwistStamped — no header needed for plain Twist

        # clamp before publishing — hard safety limit so the robot never receives
        # a command outside the safe operating range regardless of joystick input
        msg.linear.x  = self.clamp_velocity(self.linear_x,  MAX_LINEAR,  MIN_LINEAR)
        msg.angular.z = self.clamp_velocity(self.angular_z, MAX_ANGULAR, MIN_ANGULAR)

        self.linear_x  *= self.decay
        self.angular_z *= self.decay
        #prevents tiny movements to prvent robot from shaking when joystick is released, if the velocity is very small we set it to zero
        if abs(self.linear_x) < 0.01:
            self.linear_x = 0.0
        if abs(self.angular_z) < 0.01:
            self.angular_z = 0.0
        self.publisher.publish(msg)

    #update velocity from websocket data, this will be called from the websocket handler when a new message is received,
    #  we need to convert the values to float because they might come as strings from the websocket
    def update_velocity(self, linear, angular):
        # clamp on receive too so stored values are always within safe range, this way even if the websocket client sends out-of-range values, our node will never use them
        self.linear_x  = self.clamp_velocity(float(linear),  MAX_LINEAR,  MIN_LINEAR)
        self.angular_z = self.clamp_velocity(float(angular), MAX_ANGULAR, MIN_ANGULAR)

node = None

#websocket handler to receive joystick commands,will run in a separate thread and will listen for incoming websocket messages,
# when a message is received it will parse the JSON data and update the velocity of the robot using the node instance
async def handler(websocket):
    async for message in websocket:
        try:
            data = json.loads(message)
            node.update_velocity(
                data.get("linear",  0.0),
                data.get("angular", 0.0)
            )
        except Exception as e:
            print(f"[WS Error] {e}")

async def websocket_server():
    #  create server inside async function, not with asyncio.run()
    async with websockets.serve(handler, "0.0.0.0", 8765): #0.0.0.0 means any device can connect to this program(on same network), 8765 is the port number
        await asyncio.Future()  # run forever

def run_websocket_server():
    #  create a New event loop explicitly for this thread
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(websocket_server())

def main():
    global node
    rclpy.init()
    node = JoystickWebSocketNode()
    #run the websocket server in a parallel thread so it doesn't block the ROS node, we set daemon=True so that the thread will automatically close when the main program exits
    ws_thread = threading.Thread(target=run_websocket_server, daemon=True)
    ws_thread.start()
    node.get_logger().info("Joystick WebSocket Node started on ws://0.0.0.0:8765")
    node.get_logger().info(f"Velocity limits: linear={MIN_LINEAR}–{MAX_LINEAR} m/s  "
                           f"angular={MIN_ANGULAR}–{MAX_ANGULAR} rad/s")
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()