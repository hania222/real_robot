import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import asyncio     #asyncio is needed for websockets, but we will run it in a separate thread
import websockets  #library to create a websocket server
import json
import threading

class JoystickWebSocketNode(Node):
    def __init__(self):
        super().__init__('joystick_ws_node')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.05, self.publish_cmd)  # 20Hz

        #stores current velocity
        self.linear_x = 0.0
        self.angular_z = 0.0
        #decay factor to smoothly reduce velocity when no new commands are received not suddenly stop the robot
        self.decay = 0.65

    def publish_cmd(self):
        msg = Twist()
        msg.linear.x = self.linear_x
        msg.angular.z = self.angular_z

        self.linear_x *= self.decay
        self.angular_z *= self.decay
        
        #prevents tiny movements to prvent robot from shaking when joystick is released, if the velocity is very small we set it to zero
        if abs(self.linear_x) < 0.01:
            self.linear_x = 0.0
        if abs(self.angular_z) < 0.01:
            self.angular_z = 0.0

        self.publisher.publish(msg)

    #update velocity from websocket data, this will be called from the websocket handler when a new message is received, we need to convert the values to float because they might come as strings from the websocket
    def update_velocity(self, linear, angular):
        self.linear_x = float(linear)
        self.angular_z = float(angular)


node = None #global variable to store the node instance, this is needed because the websocket handler runs in a separate thread and we need to access the node instance to update the velocity

#websocket handler to receive joystick commands,will run in a separate thread and will listen for incoming websocket messages, 
# when a message is received it will parse the JSON data and update the velocity of the robot using the node instance
async def handler(websocket):
    async for message in websocket:
        try:
            data = json.loads(message)
            node.update_velocity(
                data.get("linear", 0.0),
                data.get("angular", 0.0)
            )
        except Exception as e:
            print(f"[WS Error] {e}")

async def websocket_server():
    #  create server inside async function, not with asyncio.run()
    async with websockets.serve(handler, "0.0.0.0", 8765): #0.0.0.0 means any device can connect to this program(on same network), 8765 is the port number
        await asyncio.Future()  # run forever

def run_websocket_server():
    #  create a NEW event loop explicitly for this thread
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
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()