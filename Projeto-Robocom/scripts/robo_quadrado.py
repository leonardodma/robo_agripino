#! /usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
import math
from geometry_msgs.msg import Twist, Vector3


angulo = math.pi/2
tempo = 2
w = angulo/tempo  # Velocidade angular

espaco = 1.0
v = espaco/tempo

if __name__ == "__main__":
    rospy.init_node("roda_quadrado")
    pub = rospy.Publisher("cmd_vel", Twist, queue_size=3)

    try:
        while not rospy.is_shutdown():
            vel = Twist(Vector3(v,0,0), Vector3(0,0,0))
            vel_ang = Twist(Vector3(0,0,0), Vector3(0,0,w))

                    
            pub.publish(vel)    
            rospy.sleep(tempo)
                   
            pub.publish(vel_ang)    
            rospy.sleep(tempo)

    except rospy.ROSInterruptException:
        print("Ocorreu uma exceção com o rospy")