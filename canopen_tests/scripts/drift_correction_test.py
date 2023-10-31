import rclpy
import math as m
from static_transform_broadcaster import StaticTransformBroadcaster
from odom_listener import MinimalSubscriber

from geometry_msgs.msg import TransformStamped, Vector3, Quaternion

def main():
    rclpy.init()
    transform_broadcaster = StaticTransformBroadcaster()
    odom = MinimalSubscriber()

    x = 0

    mode = input("Mode: ")
    #m - move
    #t - test
    #s - tag simulation

    if(mode == 'm'):
        while(True):
            position = Vector3()
            position.x = float(input("x-offset: "))
            position.y = float(input("y-offset: "))
            position.z = 0.0

            rotation = Quaternion()
            angle = float(input("Theta: "))
            radians = (angle/180)*m.pi
            rotation.z = m.cos(-(radians + m.pi)/2)
            rotation.w = m.sin(-(radians + m.pi)/2)

            rclpy.spin_once(odom, timeout_sec = 1)
            rotationCorrection = angleTransformCorrection(radians, odom.x, odom.y)

            position.x += rotationCorrection[0]
            position.y += rotationCorrection[1]

            transform = TransformStamped()
            transform.header.stamp = transform_broadcaster.get_clock().now().to_msg()
            transform.header.frame_id = 'map'
            transform.child_frame_id = 'odom'
            transform.transform.translation = position
            transform.transform.rotation = rotation

            transform_broadcaster.sendTransform(transform)

    if(mode == 's'):
        while(True):
            # Get and print odom data
            rclpy.spin_once(odom, timeout_sec = 1)
            print("X: " + str(odom.x) + " Y: " + str(odom.y) + " Z: " + str(odom.angz) + " W: " + str(odom.angw))

            # Simulate data matrix with inputs
            tag_position = Vector3()
            tag_position.x = float(input("tag-x: "))
            tag_position.y = float(input("tag-y: "))
            tag_position.z = 0.0

            tag_offset = Vector3()
            tag_offset.x = float(input("x-offset: "))
            tag_offset.y = float(input("y-offset: "))
            tag_offset.z = 0.0

            tag_offset_angle = Quaternion()
            angle_offset = float(input("angular-offset: "))
            angle_offset_rad = (angle_offset/180) * m.pi
            
            # Updateodom
            rclpy.spin_once(odom, timeout_sec = 1)

            # Calculate offset to correct for drift (translation)
            pos_drift = Vector3()
            pos_drift.x = -(odom.x - (tag_position.x + tag_offset.x))
            pos_drift.y = -(odom.y - (tag_position.y + tag_offset.y))

            # Calculate offset to correct for drift (orientation)
            ang_drift = Quaternion()
            odom_rad = quat_to_yaw(odom.angz, odom.angw)
            
            ang_drift.z = m.cos(-((angle_offset_rad - odom_rad) + m.pi)/2)
            ang_drift.w = m.sin(-((angle_offset_rad - odom_rad) + m.pi)/2)

            # Correct transformation after rotation
            rotationCorrection = angleTransformCorrection((angle_offset_rad - odom_rad), odom.x, odom.y)

            pos_drift.x += rotationCorrection[0]
            pos_drift.y += rotationCorrection[1]

            # Broadcast transform
            transform = TransformStamped()
            transform.header.stamp = transform_broadcaster.get_clock().now().to_msg()
            transform.header.frame_id = 'map'
            transform.child_frame_id = 'odom'
            transform.transform.translation = pos_drift
            transform.transform.rotation = ang_drift

            print(str(pos_drift.x) + " " + str(pos_drift.y) + " " + str((angle_offset - odom_rad)))

            transform_broadcaster.sendTransform(transform)
    
    if(mode ==  't'):
        print("TestMode")
        rclpy.spin_once(odom, timeout_sec = 1)
        print("Z: " + str(odom.angz)+ " W:" + str(odom.angw))
        angle = m.asin(odom.angz)
        print("Theta: " + str(angle))
        i = 0
        exit(0)


        while(True):
            rotation = Quaternion()
            position = Vector3()
            angle = i
            radians = (angle/180)*m.pi
            rotation.z = m.cos(-(radians + m.pi)/2)
            rotation.w = m.sin(-(radians + m.pi)/2)

            rclpy.spin_once(odom, timeout_sec = 1)
            rotationCorrection = angleTransformCorrection(radians, odom.x, odom.y)

            position.x += rotationCorrection[0]
            position.y += rotationCorrection[1]

            transform = TransformStamped()
            transform.header.stamp = transform_broadcaster.get_clock().now().to_msg()
            transform.header.frame_id = 'map'
            transform.child_frame_id = 'odom'
            transform.transform.translation = position
            transform.transform.rotation = rotation

            transform_broadcaster.sendTransform(transform)

            i += 0.1

    exit(0)


def angleTransformCorrection(angle_, x, y):
    angle_to_origin = m.atan2(y, x)

    if(angle_to_origin == 0):
        distance_to_origin = x
    else:
        distance_to_origin = (y) / (m.sin(angle_to_origin))

    rotation_offset_x = (m.cos(angle_ + angle_to_origin - m.pi) * distance_to_origin) + x
    rotation_offset_y = (m.sin(angle_ + angle_to_origin - m.pi) * distance_to_origin) + y

    return (rotation_offset_x, rotation_offset_y)

def quat_to_yaw(z, w):
    t1 = 2 * (w * z)
    t2 = 1 - 2 * (z * z)
    yaw = m.atan2(t1, t2)
    return yaw

if __name__ == '__main__':
    main()