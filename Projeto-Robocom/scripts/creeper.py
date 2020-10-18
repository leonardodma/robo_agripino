#! /usr/bin/env python
# -*- coding:utf-8 -*-

from __future__ import print_function, division
import rospy
import numpy as np
import cv2
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage, LaserScan
from geometry_msgs.msg import Twist, Vector3
import math
import time
from cv_bridge import CvBridge, CvBridgeError
from tf import transformations
import creepermodule


x = None
y = None

contador = 0
pula = 50

nao_bateu = True


def scaneou(dado):
    global nao_bateu
    
    print("Faixa valida: ", dado.range_min , " - ", dado.range_max )
    print("Leituras:")
    #print(np.array(dado.ranges).round(decimals=2))

    if dado.ranges[0] <= 0.15:
        nao_bateu = False
    



bridge = CvBridge()

cv_image = None
media = []
centro = []
atraso = 1.5E9 # 1 segundo e meio. Em nanossegundos

area = 0.0 # Variavel com a area do maior contorno
identifica = True

# Só usar se os relógios ROS da Raspberry e do Linux desktop estiverem sincronizados. 
# Descarta imagens que chegam atrasadas demais
check_delay = False 

# A função a seguir é chamada sempre que chega um novo frame
def roda_todo_frame(imagem):
    print("frame")
    global cv_image
    global media
    global centro
    global identifica

    now = rospy.get_rostime()
    imgtime = imagem.header.stamp
    lag = now-imgtime # calcula o lag
    delay = lag.nsecs
    print("delay ", "{:.3f}".format(delay/1.0E9))
    if delay > atraso and check_delay==True:
        print("Descartando por causa do delay do frame:", delay)
        return 
    try:
        antes = time.clock()
        cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
        #cv_image = cv2.flip(cv_image, -1) # Descomente se for robo real
        identifica = creepermodule.booleanContornos(cv_image)
        print("Identifica: {}".format(identifica))
        media, centro, maior_area =  creepermodule.identifica_rosa(cv_image)

        depois = time.clock()
        cv2.imshow("Camera", cv_image)
        
        cv2.waitKey(1)
    except CvBridgeError as e:
        print('ex', e)


if __name__=="__main__":
    rospy.init_node("Creeper")

    topico_imagem = "/kamera"
    topico_imagem = "/camera/rgb/image_raw/compressed" # Use para robo virtual
    #topico_imagem = "/raspicam/image_raw/compressed" # Use para robo real
    recebedor = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)
    print("Usando ", topico_imagem)

    velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
    recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)


    vel_1 = Twist(Vector3(0.4,0,0), Vector3(0,0,-0.4))
    vel_2 = Twist(Vector3(0.4,0,0), Vector3(0,0,0.4))
    parado = Twist(Vector3(0,0,0), Vector3(0,0,0))
    ang = Twist(Vector3(0,0,0), Vector3(0,0,0.4))

    try:
        while not rospy.is_shutdown(): 
            while identifica:
                velocidade_saida.publish(ang)
                rospy.sleep(0.1)
            
            if len(media) != 0 and len(centro) != 0:
                print("Média dos vermelhos: {0}, {1}".format(media[0], media[1]))
                print("Centro dos vermelhos: {0}, {1}".format(centro[0], centro[1]))

                if nao_bateu:
                    if (media[0] > centro[0]):
                        velocidade_saida.publish(vel_1)
                        rospy.sleep(0.1)

                    elif (media[0] < centro[0]):
                        velocidade_saida.publish(vel_2)
                        rospy.sleep(0.1)

                else:
                    velocidade_saida.publish(parado)
                    rospy.sleep(0.1)
            
            
    except rospy.ROSInterruptException:
        print("Ocorreu uma exceção com o rospy")
