"""Bounded simulated motion checks, restricted to our isolated test domain."""
import json
import math
import os
import time
import rclpy
from rclpy.parameter import Parameter
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, PointCloud2, JointState, Imu
from rosgraph_msgs.msg import Clock
from rclpy.qos import qos_profile_sensor_data
if os.environ.get('ROS_DOMAIN_ID') != '19':
    raise SystemExit('Simulation test requires ROS_DOMAIN_ID=19')
rclpy.init()
n = rclpy.create_node('k9_sim_motion_check', parameter_overrides=[Parameter('use_sim_time',value=True)])
counts = {}
last = {}
def receive(name, m):
    counts[name] = counts.get(name, 0)+1
    last[name] = m
for topic, cls in [('/odom', Odometry),('/scan', LaserScan),('/oak/points',PointCloud2),
                   ('/l_ear/scan',LaserScan),('/r_ear/scan',LaserScan),
                   ('/joint_states',JointState),('/k9/imu',Imu),('/clock',Clock)]:
    n.create_subscription(cls,topic,lambda m,t=topic: receive(t,m),qos_profile_sensor_data)
pub=n.create_publisher(TwistStamped,'/cmd_vel_nav',10)
def spin(seconds):
    end=time.monotonic()+seconds
    while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)
def pose():
    p=last['/odom'].pose.pose
    q=p.orientation
    return [p.position.x,p.position.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))]
def phase(label,seconds,x=0.,z=0.,publish=True):
    start=pose(); t0=n.get_clock().now().nanoseconds; end=time.monotonic()+30
    while (n.get_clock().now().nanoseconds-t0)/1e9<seconds:
        if time.monotonic()>end:raise RuntimeError('Simulation clock too slow/stopped during '+label)
        if publish:
            m=TwistStamped();m.header.stamp=n.get_clock().now().to_msg()
            m.twist.linear.x=x;m.twist.angular.z=z;pub.publish(m)
        spin(.05)
    v=last['/odom'].twist.twist
    print(json.dumps({'phase':label,'start':start,'end':pose(),'vx':v.linear.x,'wz':v.angular.z}),flush=True)
try:
    spin(6)
    if '/odom' not in last or '/clock' not in last:raise RuntimeError('Missing odometry or clock')
    phase('stationary',2)
    phase('forward',4,x=.10)
    phase('watchdog_stop',3,publish=False)
    phase('reverse',4,x=-.10)
    phase('stop',2)
    phase('turn',4,z=.15)
    phase('final_watchdog_stop',3,publish=False)
    print(json.dumps({'topic_counts':counts,'joint_names':last['/joint_states'].name,
                      'cloud_dimensions':[last['/oak/points'].width,last['/oak/points'].height]}),flush=True)
finally:
    m=TwistStamped();m.header.stamp=n.get_clock().now().to_msg();pub.publish(m)
    spin(.2);n.destroy_node();rclpy.shutdown()
