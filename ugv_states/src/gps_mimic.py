#!/usr/bin/env python3

import rospy
import geometry_msgs.msg as geom
import move_base_msgs.msg as move

def main():
    rospy.init_node('gps_mimic')

    cmd_pose = rospy.Publisher('/cmd_pose', move.MoveBaseGoal, queue_size=100)
    gps_topic = rospy.Publisher('/bowser2/gps', geom.PoseStamped, queue_size=100)

    gps_msg = geom.PoseStamped()
    # geom_msg = geom.PoseStamped()
    goal = move.MoveBaseGoal()

    goal.header.frame_id = "bowser2/base_link"
    goal.header.stamp = rospy.Time.now()
    # geom_msg.pose.position.x = float(input('x: '))

    # geom_msg.pose.position.y = float(input('y: '))
    # geom_msg.pose.position.z = float(input('z: '))

    # geom_msg.pose.orientation.x = float(input('x: '))
    # geom_msg.pose.orientation.y = float(input('y: '))
    # geom_msg.pose.orientation.z = float(input('z: '))
    # geom_msg.pose.orientation.w = float(input('w: '))

    while not rospy.is_shutdown():
    
       cmd_pose.publish(geom_msg)
       gps_topic.publish(gps_msg) 


if __name__ == '__main__':
    main()