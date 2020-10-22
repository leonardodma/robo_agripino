#!/usr/bin/python
# -*- coding: utf-8 -*-

import math
import cv2
import numpy as np
from matplotlib import pyplot as plt
import time
import sys


def region_of_interest(img, regiao):
    height = img.shape[0]
    width = img.shape[1]

    # Define a blank matrix that matches the image height/width.
    mask = np.zeros_like(img)
    # Retrieve the number of color channels of the image.
    channel_count = img.shape[2]
    # Create a match color with the same color channel counts.
    match_mask_color = (255,) * channel_count

    corte_pista = None
      
    if regiao == 'esquerda':
        corte_pista = [(0, height), (0, height/2), (width/2, height/2),(width/2, height), (width, height),]

    elif regiao == 'direita':
        corte_pista = [(width, height), (width, height/2), (width/2, height/2),(width/2, height), (0,height),]


    cv2.fillPoly(mask, np.array([corte_pista], np.int32), match_mask_color)
    
    # Returning the image only where mask pixels match
    masked_image = cv2.bitwise_and(img, mask)


    return masked_image


def segmenta(img):
    hsv = cv2.cvtColor(img , cv2.COLOR_BGR2HSV)  
    hsv_1, hsv_2 = np.array([0,0,240], dtype=np.uint8), np.array([255,15,255], dtype=np.uint8)
    color_mask = cv2.inRange(hsv, hsv_1, hsv_2)

    segmentado = cv2.adaptiveThreshold(cv2.morphologyEx(color_mask, cv2.MORPH_CLOSE, 
                 np.ones((10, 10))),255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,11,3.5)
    segmentado = cv2.erode(segmentado,kernel,iterations = 1)

    pista = cv2.bitwise_not(segmentado)

    return pista


def maior_linha(lines):
    a,b,c = lines.shape

    # x1, y1, x2, y2 = None, None, None, None
    maior = 0
    maior_linha = None

    for i in range(a):
        x1 = lines[i][0][0]
        y1 = lines[i][0][1]
        x2 = lines[i][0][2]
        y2 = lines[i][0][3]
        tam = (((y2-y1)**2)+((x2-x1)**2))**0.5
    
        if tam > maior:
            maior = tam
            maior_linha = lines[i][0]

    return maior_linha


def ponto_de_fuga(reta_direita, reta_esquerda):
    # IDENTIFICAÇÃO DAS RETAS:
    x1e, y1e, x2e, y2e = None, None, None, None
    x1d, y1d, x2d, y2d = None, None, None, None
    md, me = None, None

    # Identificando as linhas
    lines_esquerda = cv2.HoughLinesP(reta_esquerda, 1, np.pi/180, 30, maxLineGap=200)
    lines_direita = cv2.HoughLinesP(reta_direita, 1, np.pi/180, 30, maxLineGap=200)

    # print(lines_direita.shape)

    try:
        x1e, y1e, x2e, y2e = maior_linha(lines_esquerda)

        me = ((y1e - y2e)*1.0)/(x1e - x2e)
        cv2.line(frame, (x1e, y1e), (x2e, y2e), (0, 255, 0), 3)

        x1d, y1d, x2d, y2d = maior_linha(lines_direita)

        md = ((y1d - y2d)*1.0)/((x1d - x2d))
        cv2.line(frame, (x1d, y1d), (x2d, y2d), (255, 0, 0), 3)

        # Equação da reta esquerda:
        # y - y1e = me(x - x1e) 
        # y = me*x + y1e - me*x1e 

        # Equação da reta direita:
        # y - y1d = md(x - x1d) 
        # y = md*x + y1d - md*x1d 

        # Intersecção:
        # me*x + y1e - me*x1e  = md*x + y1d - md*x1d
        
        x, y = None, None
        if (-4 < me < -0.05) and (0.05 < md < 4):
            x = ((y1d - y1e + me*x1e - md*x1d)*1.0)/(me - md)
            y =  me*x + y1e - me*x1e
            return (int(x), int(y))
    except:
        pass

"""
cap = cv2.VideoCapture('line_following.mp4') 
lower = 0
upper = 1


while(True):
    kernel = np.ones((3, 3),np.uint8)

    # Capture frame-by-frame
    ret, frame = cap.read()

    # Cortando a imagem para a análise somente das regiões de interesse
    pista_esquerda = region_of_interest(frame, 'esquerda',)
    pista_direita = region_of_interest(frame, 'direita',)  

    pista_direita_segmentado = segmenta(pista_direita)
    pista_esquerda_segmentado = segmenta(pista_esquerda)

    # Ponto de Fuga
    cv2.circle(frame, ponto_de_fuga(pista_direita_segmentado, pista_esquerda_segmentado), 4, (0, 0, 255), 6)

    cv2.imshow("Ponto de Fuga", frame)

    if cv2.waitKey(1) &  0xFF == ord('q'):
        break


#  When everything done, release the capture
cap.release()
cv2.destroyAllWindows() 
"""