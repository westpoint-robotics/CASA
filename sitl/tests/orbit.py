import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import asyncio
from mavsdk import System
import mavsdk

import json

async def run(node):
    drone = System()
    await drone.connect(system_address="udp://:14540")    
    await asyncio.ensure_future(orbit(drone, node))
    asyncio.ensure_future(print_position(drone, node))

async def orbit(drone, node):
    await drone.action.arm()
    await asyncio.sleep(1.0)
    await drone.action.takeoff()
    await asyncio.sleep(1.0)
    await drone.action.do_orbit(100, 1, mavsdk.action.OrbitYawBehavior(2), 41.3932014,-73.9565756, 1000)
    
async def print_position(drone, node):
    async for position in drone.telemetry.position():
        node.gposition = position
        print(f"position: {position}")

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'sitl_orbit', 10)
        timer_period = 0.5  # seconds
        self.timer= self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.gposition = []        
        self.battery_percent = None
        self._event_loop = asyncio.get_running_loop()

    async def timer_callback(self):
        msg = String()
        mdata = {}
        mdata['name'] = "sitl_monitor"
        mdata['sample'] = self.i
        mdata['position'] = str(self.gposition)         
        msg.data = json.dumps(mdata)
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


async def spin_once(minimal_publisher):
    rclpy.spin_once(minimal_publisher, timeout_sec=0)


async def run_ros2(minimal_publisher):
    while True:
        await spin_once(minimal_publisher)
        await asyncio.sleep(0.0)


async def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    task_ros2 = asyncio.create_task(run_ros2(minimal_publisher))
    task_mavsdk = asyncio.create_task(run(minimal_publisher))
    await asyncio.gather(task_ros2, task_mavsdk)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

asyncio.run(main())