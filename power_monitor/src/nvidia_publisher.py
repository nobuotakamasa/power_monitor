
import rclpy
from std_msgs.msg import Float32
import subprocess
import re
import sys

import socket

hostname = socket.gethostname().replace('-', '_')
print("Modified Hostname:", hostname)

interval = "1000"
if len(sys.argv) > 1:
    interval = sys.argv[1]
print("interval time " + interval + " msec")
topic_name = '/ecu/'+hostname+"/gpu"

#format 2024/08/18 21:58:35.907, 4.69
def get_power(output):
    numbers = re.findall(r'\d+',output)
    p = numbers[-2:]
    power = float(p[0]) + float(p[1]) / 1000.0
    return power

def main():
    rclpy.init()
    node = rclpy.create_node('nvidia_publisher')
    #nvidia-smi --query-gpu=timestamp,power.draw --format=csv,noheader,nounits -lms 1000
    command = ["nvidia-smi", "--query-gpu=timestamp,power.draw", "-lms", interval,
               "--format=csv,noheader,nounits"]
    #node.get_logger().info(f'interval: {interval}')
    process = subprocess.Popen(command, stdout = subprocess.PIPE, stderr=subprocess.PIPE)

    # Create a publisher to publish the output
    publisher = node.create_publisher(Float32, topic_name, 10)
    while rclpy.ok():
        output = str(process.stdout.readline().strip())
        #node.get_logger().info(f'get: {output}')
        power = get_power(output)
        msg = Float32(data=power)
        node.get_logger().info(f'Publishing: {power}')
        publisher.publish(msg)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


