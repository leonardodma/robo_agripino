#! /usr/bin/env python
# -*- coding:utf-8 -*-


import numpy as np
import math
import cv2
import time
import auxiliar as aux


def identifica_creeper(frame, creeper_color):
    segmentado_cor = None
    frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # segmentação para trabalhar com as cores dos creepers da pista 
    if creeper_color == "rosa":
        cor_menor, cor_maior = aux.ranges("#e60c69")
        segmentado_cor = cv2.inRange(frame_hsv, cor_menor, cor_maior)
    
    elif creeper_color == "azul":
        cor_menor, cor_maior = aux.ranges("#0000ff")
        segmentado_cor = cv2.inRange(frame_hsv, cor_menor, cor_maior)

    elif creeper_color == "vermelho":
        cor_menor = np.array([0, 180, 135])
        cor_maior = np.array([2, 255, 255])
        segmentado_cor = cv2.inRange(frame_hsv, cor_menor, cor_maior)

        cor_menor = np.array([178, 180, 135])
        cor_maior = np.array([180, 255, 255])
        segmentado_cor += cv2.inRange(frame_hsv, cor_menor, cor_maior)
    
    else: # creeper_color == "verde":
        cor_menor, cor_maior = aux.ranges("#00ff00")
        segmentado_cor = cv2.inRange(frame_hsv, cor_menor, cor_maior)


    # Note que a notacão do numpy encara as imagens como matriz, portanto o enderecamento é
    # linha, coluna ou (y,x)
    # Por isso na hora de montar a tupla com o centro precisamos inverter, porque 
    centro = (frame.shape[1]//2, frame.shape[0]//2)


    def cross(img_rgb, point, color, width,length):
        cv2.line(img_rgb, (point[0] - length/2, point[1]),  (point[0] + length/2, point[1]), color ,width, length)
        cv2.line(img_rgb, (point[0], point[1] - length/2), (point[0], point[1] + length/2),color ,width, length) 


    # Encontramos os contornos na máscara e selecionamos o de maior área
    contornos, arvore = cv2.findContours(segmentado_cor.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 
    
    maior_contorno = None
    maior_contorno_area = 0

    for cnt in contornos:
        area = cv2.contourArea(cnt)
        if area > maior_contorno_area:
            maior_contorno = cnt
            maior_contorno_area = area

    def booleanContornos(maior_contorno):
        if maior_contorno_area > 100:
            return True
        else:
            return False

    media = None
    # Encontramos o centro do contorno fazendo a média de todos seus pontos.
    if not maior_contorno is None :
        cv2.drawContours(frame, [maior_contorno], -1, [0, 255, 0], 5)
        maior_contorno = np.reshape(maior_contorno, (maior_contorno.shape[0], 2))
        media = maior_contorno.mean(axis=0)
        media = media.astype(np.int32)
        cv2.circle(frame, (media[0], media[1]), 5, [0, 255, 0])
        cross(frame, centro, [0, 255,0], 1, 17)
    else:
        media = (0, 0)

    cv2.imshow('seg', segmentado_cor)
    cv2.waitKey(1)

    return media, centro, maior_contorno_area, booleanContornos(maior_contorno)