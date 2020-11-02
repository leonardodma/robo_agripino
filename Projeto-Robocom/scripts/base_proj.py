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
media_pista = []
centro_pista = []
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
contador = 0
pula = 50
alpha = 0

frame = "camera_link"
# frame = "head_camera"  # DESCOMENTE para usar com webcam USB via roslaunch tag_tracking usbcam

tfl = 0

media_creeper = []
centro_creeper = []

# Variáveis booleanas
tf_buffer = tf2_ros.Buffer()
nao_bateu = True
identifica_contorno_pista = True
identifica_creeper = False
# Marcação para guardar a posição anterior à identificação do creeper
flag = True


def scaneou(dado):
    global nao_bateu
    # 25cm
    print('distancia: ', dado.ranges[0])
    if dado.ranges[0] <= 0.35:
        nao_bateu = False


def recebe_odometria(data):
    global x
    global y
    global contador

    x = data.pose.pose.position.x
    y = data.pose.pose.position.y

    quat = data.pose.pose.orientation
    lista = [quat.x, quat.y, quat.z, quat.w]
    angulos = np.degrees(transformations.euler_from_quaternion(lista))    

    if contador % pula == 0:
        print("Posicao (x,y)  ({:.2f} , {:.2f}) + angulo {:.2f}".format(x, y,angulos[2]))
    contador = contador + 1


def go_to(x1, y1, v, w, pub):
    global alpha 
    global x
    global y
    global dist
    global zero

    x0 = x
    y0 = y
    deltay = y1-y0
    deltax = x1-x0

    dist = ((deltax)**2 + (deltay)**2)**(1/2)
    zero = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))

    while dist > 0.3:
        teta = math.atan2(deltay, deltax)
        angulo = teta - alpha
        tempo = abs(angulo)/w

        if angulo > 0:
            velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0, w))
        
        else:
            velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0, -w))


        pub.publish(velocidade)
        rospy.sleep(tempo)
        pub.publish(zero)
        rospy.sleep(0.1)

        # Translação
        tempo = dist/v
        velocidade = Twist(Vector3(v, 0, 0), Vector3(0, 0, 0))
        pub.publish(velocidade)
        rospy.sleep(tempo)

        pub.publish(zero)
        rospy.sleep(0.1)

        x0 = x
        y0 = y
        deltay = y1-y0
        deltax = x1-x0
        dist = ((deltax)**2 + (deltay)**2)**(1/2)
        print(dist)


# A função a seguir é chamada sempre que chega um novo frame
def roda_todo_frame(imagem):
    global cv_image
    global media_pista
    global centro_pista
    global resultados
    global identifica_contorno_pista

    global media_creeper
    global centro_creeper
    global identifica_creeper

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
        # cv_image = cv2.flip(cv_image, -1) # Descomente se for robo real
        media_pista, centro_pista, maior_area, identifica_contorno_pista =  center_mass.identifica_pista(cv_image)
        media_creeper, centro_creeper, maior_area_creeper, identifica_creeper =  creeper.identifica_creeper(cv_image, "rosa")

        cv2.imshow("cv_image", cv_image)
        cv2.waitKey(1)

    except CvBridgeError as e:
        print('ex', e)
    path = rospack.get_path('Projeto-Robocom')


if __name__=="__main__":
    rospy.init_node("cor")

    topico_imagem = "/camera/image/compressed"
    # topico_imagem = "/raspicam/image_raw/compressed" # Use para robo real

    recebedor = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)

    print("Usando ", topico_imagem)

    velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
    ref_odometria = rospy.Subscriber("/odom", Odometry, recebe_odometria)
    recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)

    tfl = tf2_ros.TransformListener(tf_buffer) #conversao do sistema de coordenadas 
    tolerancia = 25

    # Exemplo de categoria de resultados
    # [('chair', 86.965459585189819, (90, 141), (177, 265))]

    try:
        w = 0.25
        v = 0.25
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
            
            # Marcação para guardar a posição anterior à identificação do creeper
            #flag = True

            if not identifica_creeper:
                while not identifica_contorno_pista:
                    velocidade_saida.publish(virar)
                    rospy.sleep(0.1)

                if nao_bateu:
                    try:
                        if (media_pista[0] > centro_pista[0]):
                            velocidade_saida.publish(vel_1)
                            rospy.sleep(0.1)
                        elif (media_pista[0] < centro_pista[0]):
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

            else:
                #ponto =  None
                if flag:
                    ponto = (x,y)
                    flag = False
                    print('Hasta la vista babeeeeee! Ponto: ', ponto) 

                if nao_bateu: 
                    print('rosa here I goooo')
                    if (media_creeper[0] > centro_creeper[0]):
                        velocidade_saida.publish(vel_1)
                        rospy.sleep(0.1)
                    elif (media_creeper[0] < centro_creeper[0]):
                        velocidade_saida.publish(vel_2)
                        rospy.sleep(0.1)
                else:
                    print('mals ae')
                    go_to(ponto[0], ponto[1], v, w, velocidade_saida)
                    flag = True
                    print('voltando ao ponto inicial')



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