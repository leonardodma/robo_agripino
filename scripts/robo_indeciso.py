#! /usr/bin/env python
# -*- coding:utf-8 -*-


import rospy
import numpy as np
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan


nao_bateu = True


def scaneou(dado):
    global nao_bateu
    
    print("Faixa valida: ", dado.range_min , " - ", dado.range_max )
    print("Leituras:")
    #print(np.array(dado.ranges).round(decimals=2))

    if dado.ranges[0] <= 1:
        nao_bateu = False
    elif dado.ranges[0] <= 1.2:
        nao_bateu = True



if __name__=="__main__":
    rospy.init_node("robo_indeciso")
    velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 3 )
    recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)

    try:
        while not rospy.is_shutdown():
            if nao_bateu: 
                velocidade = Twist(Vector3(0.50, 0, 0), Vector3(0, 0, 0))
                
            else:
                print('Entrou')
                velocidade = Twist(Vector3(-0.50, 0, 0), Vector3(0, 0, 0))

            velocidade_saida.publish(velocidade)
            rospy.sleep(2)

    except rospy.ROSInterruptException:
        print("Ocorreu uma exceção com o rospy")
