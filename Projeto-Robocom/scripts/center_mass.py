#!/usr/bin/python
# -*- coding: utf-8 -*-

import cv2
import numpy as np 


def identifica_pista(bgr):
    centro = (bgr.shape[1]//2, bgr.shape[0]//2)

    # Valores para amarelo usando um color picker
    # mascara amarela
    low = np.array([22, 50, 50],dtype=np.uint8)
    high = np.array([36, 255, 255],dtype=np.uint8)
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, low, high)
    

    # Encontramos os contornos na máscara e selecionamos o de maior área
    contornos, arvore = cv2.findContours(mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 
    

    maior_contorno = None
    maior_contorno_area = 0

    for cnt in contornos:
        area = cv2.contourArea(cnt)
        if area > maior_contorno_area:
            maior_contorno = cnt
            maior_contorno_area = area

    media = None
    if not maior_contorno is None:
        try:
            p = center_of_mass(maior_contorno) # centro de massa
            crosshair(bgr, p, 20, (128,128,0))
            maior_contorno = np.reshape(maior_contorno, (maior_contorno.shape[0], 2))
            media = maior_contorno.mean(axis=0)
            media = media.astype(np.int32)
        
        except:
            p = centro
            crosshair(bgr, p, 20, (128,128,0))

    def booleanContornos(maior_contorno):
        if maior_contorno_area > 100:
            return True
        else:
            return False

    
    return media, centro, maior_contorno_area, booleanContornos(maior_contorno)


# Função centro de massa baseada na aula 02  https://github.com/Insper/robot202/blob/master/aula02/aula02_Exemplos_Adicionais.ipynb
# Esta função calcula centro de massa de máscara binária 0-255 também, não só de contorno
def center_of_mass(mask):
    """ Retorna uma tupla (cx, cy) que desenha o centro do contorno"""
    M = cv2.moments(mask)

    # Usando a expressão do centróide definida em: https://en.wikipedia.org/wiki/Image_moment
    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])
    return [int(cX), int(cY)]



def crosshair(img, point, size, color):
    """ Desenha um crosshair centrado no point.
        point deve ser uma tupla (x,y)
        color é uma tupla R,G,B uint8
    """
    x,y = point
    cv2.line(img,(x - size,y),(x + size,y),color,5)
    cv2.line(img,(x,y - size),(x, y + size),color,5)


def center_of_mass_region(mask, x1, y1, x2, y2):
    # Para fins de desenho
    mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    clipped = mask[y1:y2, x1:x2]
    c = center_of_mass(clipped)
    c[0]+=x1
    c[1]+=y1
    crosshair(mask_bgr, c, 10, (0,0,255))
    return mask_bgr
