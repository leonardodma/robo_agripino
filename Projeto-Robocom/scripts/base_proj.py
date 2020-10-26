#! /usr/bin/env python
# -*- coding:utf-8 -*-

from __future__ import print_function, division
import rospy
import numpy as np
import numpy
import tf
import math
import cv2
import time
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage, LaserScan
from cv_bridge import CvBridge, CvBridgeError
from numpy import linalg
from tf import transformations
from tf import TransformerROS
import tf2_ros
from geometry_msgs.msg import Twist, Vector3, Pose, Vector3Stamped
import rospkg
from nav_msgs.msg import Odometry
from std_msgs.msg import Header



import visao_module
import center_mass
import creeper


bridge = CvBridge()
rospack = rospkg.RosPack()


cv_image = None
media = []
centro = []
atraso = 1.5E9 # 1 segundo e meio. Em nanossegundos


area = 0.0 # Variavel com a area do maior contorno

# Só usar se os relógios ROS da Raspberry e do Linux desktop estiverem sincronizados. 
# Descarta imagens que chegam atrasadas demais
check_delay = False 

resultados = [] # Criacao de uma variavel global para guardar os resultados vistos

x = 0
y = 0
z = 0 
id = 0

frame = "camera_link"
# frame = "head_camera"  # DESCOMENTE para usar com webcam USB via roslaunch tag_tracking usbcam

tfl = 0

tf_buffer = tf2_ros.Buffer()
nao_bateu = True
identifica_contorno_pista = True


def scaneou(dado):
    global nao_bateu
    if dado.ranges[0] <= 0.25:
        nao_bateu = False


# A função a seguir é chamada sempre que chega um novo frame
def roda_todo_frame(imagem):
    global cv_image
    global media
    global centro
    global resultados
    global identifica_contorno_pista

    now = rospy.get_rostime()
    imgtime = imagem.header.stamp
    lag = now-imgtime # calcula o lag
    delay = lag.nsecs
    # print("delay ", "{:.3f}".format(delay/1.0E9))
    if delay > atraso and check_delay==True:
        print("Descartando por causa do delay do frame:", delay)
        return 
    try:
        antes = time.clock()
        temp_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
        # Note que os resultados já são guardados automaticamente na variável
        # chamada resultados
        centro, saida_net, resultados =  visao_module.processa(temp_image)        
        for r in resultados:
            # print(r) - print feito para documentar e entender
            # o resultado            
            pass

        depois = time.clock()
        # Desnecessário - Hough e MobileNet já abrem janelas

        cv_image = saida_net.copy()
        media, centro, maior_area, identifica_contorno_pista =  center_mass.identifica_pista(cv_image)

        cv2.imshow("cv_image", cv_image)
       
        
        cv2.waitKey(1)

    except CvBridgeError as e:
        print('ex', e)
    path = rospack.get_path('Projeto-Robocom')


if __name__=="__main__":
    rospy.init_node("cor")

    topico_imagem = "/camera/image/compressed"

    recebedor = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)

    print("Usando ", topico_imagem)

    velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
    recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)

    tfl = tf2_ros.TransformListener(tf_buffer) #conversao do sistema de coordenadas 
    tolerancia = 25

    # Exemplo de categoria de resultados
    # [('chair', 86.965459585189819, (90, 141), (177, 265))]

    try:
        w = 0.4
        v = 0.4
        # Inicializando - por default gira no sentido anti-horário
        vel_1 = Twist(Vector3(v,0,0), Vector3(0,0,-w))
        vel_2 = Twist(Vector3(v,0,0), Vector3(0,0,w))
        vel = Twist(Vector3(v,0,0), Vector3(0,0,0))
        parado = Twist(Vector3(0,0,0), Vector3(0,0,0))
        virar = Twist(Vector3(0,0,0), Vector3(0,0,w))
        
        dist = 0.8
        tempo1 = dist/v

        # w = dteta/dt
        angulo = math.pi
        tempo2 = angulo/w

        
        while not rospy.is_shutdown():
            for r in resultados:
                print(r)

            while not identifica_contorno_pista:
                velocidade_saida.publish(virar)
                rospy.sleep(0.1)

            if nao_bateu:
                try:
                    if (media[0] > centro[0]):
                        velocidade_saida.publish(vel_1)
                        rospy.sleep(0.1)
                    elif (media[0] < centro[0]):
                        velocidade_saida.publish(vel_2)
                        rospy.sleep(0.1)
                    else:
                        velocidade_saida.publish(parado)
                        rospy.sleep(0.1)
                except:
                    velocidade_saida.publish(virar)
                    rospy.sleep(0.1)

            else:
                velocidade_saida.publish(virar)
                rospy.sleep(tempo2)

                velocidade_saida.publish(vel)
                rospy.sleep(tempo1)

        """
        estado = "frente"
        estado = "frente_direita"
        estado = "frente_esquerda"
        estado = "parado"
        estado = "virando_direita"

        def maquina_de_estados(media, centro, nao_bateu, identifica_contorno_pista):
            
            if nao_bateu:
                if (media[0] > centro[0]):
                    estado = "frente_direita"
                if (media[0] < centro[0]):
                    estado = "frente_direita"
            else:
                estado = "

        """

    except rospy.ROSInterruptException:
        print("Ocorreu uma exceção com o rospy")