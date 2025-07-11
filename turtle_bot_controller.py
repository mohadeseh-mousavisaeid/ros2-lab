#!/usr/bin/env python

import rospy
import random
from turtlesim.srv import Spawn, SpawnRequest
from turtlesim.msg import Pose



rospy.init_node('turtle spawner', anonymous=True)


# Wait for the spawn service
rospy.wait_for_service('/spawn')
spawn_srv = rospy.ServiceProxy('/spawn', Spawn)


turtle_count = 1


def pose_callback(pose):

    if pose.x <= 0.5 or pose.x >=10.5 or pose.y <= 0.5 or pose.y >=10.5:
        rospy.loginfo(" Turtle hit the wall! Spawning a new turtle")


        # Generate a unique name
        turtle_count += 1
        new_turtle_name = f"turtle{turtle_count}"


        # Random position (within safe limits)
        new_x = random.uniform(2.0,9.0)
        new_y = random.uniform(2.0,9.0)
        new_theta = random.uniform(0,6.28)


        #Spawned a new turtle

        try:
            spawn_srv(new_x, new_y, new_theta,new_turtle_name)
            rospy.loginfo(f"New turtle spawned: {new_turtle_nmae} at ({new_x}, {new_y})")

        except Exception as e:
            rospy.logwarn(e)

if _name_ == '_main_' :
    #Subscribe to turtle1's position
    pose_sub = rospy.Subscriber('/turtle1/pose', Pose, callback=pose_callback)

    try: 
        TurtleSpawner()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass