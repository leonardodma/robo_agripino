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


# Import de outros funções importantes 
import visao_module
import center_mass
import creeper


bridge = CvBridge() #arquivo ros pra abrir o cv_img
cv_image = None # Cv Image
rospack = rospkg.RosPack() 


#INICIALIZAÇÃO DE VARIÁVEIS GLOBAIS

#Objetivo
#         cor   id  estacao
goal = ("rosa", 13, "bird")

# Def center_mass
media_pista = []
centro_pista = []
identifica_contorno_pista = True
identifica_creeper = False

# Atrasos e delays - Descarta imagens que chegam atrasadas demais
# Só usar se os relógios ROS da Raspberry e do Linux desktop estiverem sincronizados.  
atraso = 1.5E9 # 1 segundo e meio. Em nanossegundos
check_delay = False

# MobileNet
resultados = [] # Criacao de uma variavel global para guardar os resultados vistos

# Odometria
x = 0
y = 0
z = 0 
alpha = 0

# Aruco
id = 0

# Conversão do sistema de coordenadas
tfl = 0
tf_buffer = tf2_ros.Buffer()

# Laser scan
nao_bateu = True

# Inicializando velocidades - por default gira no sentido anti-horário
w = 0.3
v = 0.3

vel_direita = Twist(Vector3(v,0,0), Vector3(0,0,-w))
vel_esquerda = Twist(Vector3(v,0,0), Vector3(0,0,w))
vel_frente = Twist(Vector3(v,0,0), Vector3(0,0,0))
vel_parado = Twist(Vector3(0,0,0), Vector3(0,0,0))
vel_girando = Twist(Vector3(0,0,0), Vector3(0,0,w))


# Máquina de Estado
estado = None
subestado = None

# Verificação se creeper já foi identificado
identificado = False

# Marcação para guardar a posição anterior à identificação do creeper
flag = True


# FUNÇÕS A SEREM UTILIZADAS

def scaneou(dado):
    global nao_bateu
    # 40cm
    if dado.ranges[0] <= 0.40:
        nao_bateu = False
    else:
        nao_bateu = True


def recebe_odometria(data):
    global x
    global y
    global alpha

    x = data.pose.pose.position.x
    y = data.pose.pose.position.y

    quat = data.pose.pose.orientation
    lista = [quat.x, quat.y, quat.z, quat.w]
    angulos_rad = transformations.euler_from_quaternion(lista)

    alpha = angulos_rad[2] # mais facil se guardarmos alpha em radianos
    

def go_to(x2, y2, pub):
    # Odometria
    global alpha 
    global x
    global y

    # Velocidades 
    global v
    global w
    global vel_girando
    global vel_frente
    
    # calcular theta
    theta = math.atan2(y2-y, x2-x)

    # ângulo que o robô deve virar para se ajustar com o ponto
    angulo = theta - alpha

    # girar theta - alpha para a esquerda
    tempo = angulo / w


    # corrigir o ângulo
    pub.publish(vel_girando)
    rospy.sleep(tempo)

    # andar 0.8 metros
    tempo_2 = 0.8 / v
    pub.publish(vel_frente)
    rospy.sleep(tempo_2)

    
def meia_volta():
    global vel_girando
    global vel_frente

    tempo1 = 0.8 /v

    # w = dteta/dt
    angulo = math.pi
    tempo2 = angulo/w

    velocidade_saida.publish(vel_girando)
    rospy.sleep(tempo2)

    velocidade_saida.publish(vel_frente)
    rospy.sleep(tempo1)


def direcao_robo_pista():
    global subestado

    if subestado == 'frente':
        velocidade_saida.publish(vel_frente)

    if subestado == 'esquerda':
        velocidade_saida.publish(vel_esquerda)

    if subestado == 'direita':
        velocidade_saida.publish(vel_direita)

    if subestado == 'final da pista':
        meia_volta()


def identifica_id():
    global id
    global vel_parado

    velocidade_saida.publish(vel_parado)
    rospy.sleep(5)



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

    # Para robô real
    now = rospy.get_rostime()
    imgtime = imagem.header.stamp
    lag = now-imgtime # calcula o lag
    delay = lag.nsecs

    if delay > atraso and check_delay==True:
        print("Descartando por causa do delay do frame:", delay)
        return 
    try:
        antes = time.clock()
        temp_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
        # Note que os resultados já são guardados automaticamente na variável chamada resultados
        centro, saida_net, resultados =  visao_module.processa(temp_image)        
        for r in resultados:
            # print(r) - print dos resultados da mobilebnet
            pass

        depois = time.clock()
        # Desnecessário - Hough e MobileNet já abrem janelas

        cv_image = saida_net.copy()
        # cv_image = cv2.flip(cv_image, -1) # Descomente se for robo real
        media_pista, centro_pista, maior_area, identifica_contorno_pista =  center_mass.identifica_pista(temp_image)
        media_creeper, centro_creeper, maior_area_creeper, identifica_creeper =  creeper.identifica_creeper(temp_image, goal[0])

        cv2.imshow("cv_image", temp_image)
        cv2.waitKey(1)

    except CvBridgeError as e:
        print('ex', e)
    path = rospack.get_path('Projeto-Robocom')


if __name__=="__main__":
    rospy.init_node("agripino") 

    topico_imagem = "/camera/image/compressed"
    # topico_imagem = "/raspicam/image_raw/compressed" # Use para robo real

    recebedor = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)

    print("Usando ", topico_imagem)

    velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
    ref_odometria = rospy.Subscriber("/odom", Odometry, recebe_odometria)
    recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)

    tfl = tf2_ros.TransformListener(tf_buffer) #conversao do sistema de coordenadas 
    
    # Exemplo de categoria de resultados
    # [('chair', 86.965459585189819, (90, 141), (177, 265))]


    try:
        estado = 'procurando pista'

        while not rospy.is_shutdown():
            # for r in resultados:
            #    print(r)

            if estado == 'procurando pista':
                # Se eu nao identifico creeper eu procuro contorno
                if not identifica_creeper: 
                    while not identifica_contorno_pista:
                        velocidade_saida.publish(vel_girando)
                        rospy.sleep(0.01)
                    # Se eu identificar o contorno mudo meu estado e passo a seguir a pista    
                    estado = 'segue pista'

                # Se eu acho o creeper mudo o estado
                else: 
                    estado = 'creeper a la vista'

            if estado == 'segue pista':

                #precisa colocar a centralizacao da pista e o atualizar o subestado de acordo
                #precisa atualizar estado caso veja um creeper
                #precisa atualizar estado caso nao veja mais a pista
                try:
                    if nao_bateu:
                        print("Encontrei pista")
                        if (media_pista[0] > centro_pista[0]):
                            subestado = 'direita'
                        elif (media_pista[0] < centro_pista[0]):
                            subestado = 'esquerda'
                        else:
                            subestado = 'frente'

                    else: #quando ve o final da pista vira 180 e anda 0.8 pra frente
                        print("Encontrei algum obstáculo")
                        subestado = 'final da pista'
                except:
                    pass


                # Mudança de estado
                if not identifica_contorno_pista:
                    estado = 'procurando pista'

                if identifica_creeper and identificado==False:
                    estado = 'creeper a la vista'  

                # Chama a função direção_robo_pista, que seta a velocidade do robô com base nos substados
                direcao_robo_pista()
                rospy.sleep(0.01)  

            # Creeper foi identificado
            if estado == 'creeper a la vista':
                identificado = True

                if flag:
                    ponto = (x,y)
                    print("Ponto gravado:", ponto)
                    flag = False
                    print('Hasta la vista babeeeeee! Ponto: ', ponto) 

                if nao_bateu: 
                    subestado = 'segue creeper'
                    print('rosa here I goooo')
        
                else:
                    # Chama a função para identificar o Id do Creeper com o Aruco
                    identifica_id()
                    subestado = 'retorna pista'
                    print('mals ae')
                

                if subestado == 'segue creeper':
                    if (media_creeper[0] > centro_creeper[0]):
                        velocidade_saida.publish(vel_direita)
                    elif (media_creeper[0] < centro_creeper[0]):
                        velocidade_saida.publish(vel_esquerda)

                    rospy.sleep(0.01)

                if subestado == 'pega creeper':
                    # Colocar aqui código para identificar Id do creeper
                    pass
                
                if subestado == 'retorna pista':
                    go_to(ponto[0], ponto[1], velocidade_saida)
                    identifica_creeper = False
                    estado = 'procurando pista'

            print("#####################################")
            print('estado: ', estado)
            print('subestado: ', subestado)

    except rospy.ROSInterruptException:
        print("Ocorreu uma exceção com o rospy")