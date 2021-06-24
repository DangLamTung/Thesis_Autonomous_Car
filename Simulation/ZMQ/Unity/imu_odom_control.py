
# license removed for brevity
import rospy

import math
from math import sin, cos, pi

from std_msgs.msg import String
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

from advanced_uart import *


def callback(data):
    print(data.linear,data.angular)
    send_uart(data.linear.x,data.angular.z)

def main():
    global ser
    rospy.init_node('imu_odom_control')
    pub_imu = rospy.Publisher('imu', Imu, queue_size=10)



    rospy.Subscriber("cmd_vel", Twist, callback)
    rate = rospy.Rate(10) # 10hz
    
    odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)

    odom_broadcaster = tf.TransformBroadcaster()

    x = 0.0
    y = 0.0
    th = 0.0

    vx = 0
    vy = 0
    vth = 0

    current_time = rospy.Time.now()
    last_time = rospy.Time.now()

    while not rospy.is_shutdown():

        data = advance_uart()
        
        imu_data = Imu()
        try:
            vx = data[0]/1000
            vy = data[1]/1000
            vth = data[6]
            print(vx/1000,vy/1000,data[2])
            current_time = rospy.Time.now()
            imu_data.header.stamp = current_time
            imu_data.header.frame_id = "odom"
            imu_data.angular_velocity.x = data[4]
            imu_data.angular_velocity.y = data[5]
            imu_data.angular_velocity.z = data[6]
            imu_data.linear_acceleration.x = data[7]
            imu_data.linear_acceleration.y = data[8]
            imu_data.linear_acceleration.z = data[9]


            
            # compute odometry in a typical way given the velocities of the robot
            dt = (current_time - last_time).to_sec()
            delta_x = (vx * cos(th) - vy * sin(th)) * dt
            delta_y = (vx * sin(th) + vy * cos(th)) * dt
            delta_th = vth * dt

            x += delta_x
            y += delta_y
            th += delta_th

            # since all odometry is 6DOF we'll need a quaternion created from yaw
            odom_quat = tf.transformations.quaternion_from_euler(0, 0, data[2])

            # first, we'll publish the transform over tf
            odom_broadcaster.sendTransform(
                (x, y, 0.),
                odom_quat,
                current_time,
                "base_link",
                "odom"
            )

            # next, we'll publish the odometry message over ROS
            odom = Odometry()
            odom.header.stamp = current_time
            odom.header.frame_id = "odom"

            # set the position
            odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

            # set the velocity
            odom.child_frame_id = "base_link"
            odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

            # publish the message
            odom_pub.publish(odom)
            # print(odom)
            pub_imu.publish(imu_data)
            last_time = current_time
        except Exception as e:
            print(e)
        
        rate.sleep()
    # rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass