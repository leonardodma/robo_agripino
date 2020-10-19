import cv2
import time as t
import sys
import math
import auxiliar as aux
import numpy as np

if (sys.version_info > (3, 0)):
    # Modo Python 3
    import importlib
    importlib.reload(aux) # Para garantir que o Jupyter sempre relê seu trabalho
else:
    # Modo Python 2
    reload(aux)


#####################################################################################################################################
#funcoes úteis

def hsv_hists(img, plt):
    """
        Plota o histograma de cada um dos canais HSV
        img - imagem HSV
        plt - objeto matplotlib
    """
    plt.figure(figsize=(20,10));
    img_h = img[:,:,0]
    img_s = img[:,:,1]
    img_v = img[:,:,2]
    histo_plot(img_h, "r","H", plt);
    histo_plot(img_s, "g","S", plt);
    histo_plot(img_v, "b","V", plt);

def make_hist(img_255, c, label, plt):
    """ img_255 - uma imagem com 3 canais de 0 até 255
        c a cor do plot
        label - o label do gráfico
        plt - matplotlib.pyplot
    """
    hist,bins = np.histogram(img_255.flatten(),256,[0,256])
    cdf = hist.cumsum()
    cdf_normalized = cdf * hist.max()/ cdf.max()

    # plt.plot(cdf_normalized, color = c)
    plt.hist(img_255.flatten(),256,[0,256], color = c)
    plt.xlim([0,256])
    plt.legend(label, loc = 'upper left')
    plt.plot()

def histo_plot(img, cor, label, plt):
    """
        img - imagem
        cor - cor
        plt - matplotlib.pyplot object

    """
    plt.figure(figsize=(10,5))
    make_hist(img, cor, label, plt)
    plt.show()
    plt.figure(figsize=(10,5))
    plt.imshow(img, cmap="Greys_r")#, vmin=0, vmax=255)
    plt.title(label)

def center_of_contour(contorno):
    """ Retorna uma tupla (cx, cy) que desenha o centro do contorno"""
    M = cv2.moments(contorno)
    # Usando a expressão do centróide definida em: https://en.wikipedia.org/wiki/Image_moment
    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])
    return (int(cX), int(cY))
    
def crosshair(img, point, size, color):
    """ Desenha um crosshair centrado no point.
        point deve ser uma tupla (x,y)
        color é uma tupla R,G,B uint8
    """
    x,y = point
    cv2.line(img,(x - size,y),(x + size,y),color,5)
    cv2.line(img,(x,y - size),(x, y + size),color,5)
    
font = cv2.FONT_HERSHEY_SIMPLEX

def texto(img, a, p):
    """Escreve na img RGB dada a string a na posição definida pela tupla p"""
    cv2.putText(img, str(a), p, font,1,(0,50,100),2,cv2.LINE_AA)
    
def auto_canny(image, sigma=0.33):
    # compute the median of the single channel pixel intensities
    v = np.median(image)

    # apply automatic Canny edge detection using the computed median
    lower = int(max(0, (1.0 - sigma) * v))
    upper = int(min(255, (1.0 + sigma) * v))
    edged = cv2.Canny(image, lower, upper)

    # return the edged image
    return edged
####################################################################################################################################


if __name__ == "__main__":
    if len(sys.argv) > 1:
        arg = sys.argv[1]
        try:
            input_source=int(arg) # se for um device
        except:
            input_source=str(arg) # se for nome de arquivo
    else:
        input_source = 0
        
    # iniciando a captura
    cap = cv2.VideoCapture('line_following.mp4')
    
    while(True):
        # Capture frame-by-frame
        ret, frame = cap.read()
        if ret == False:
            print("Codigo de retorno FALSO - fim do video")
        
        
        # convertendo os frames para rgb e hsv e gray
        img_bgr = frame.copy()
        img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
        img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
        img_gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
        
        #Definindo os limites do inrange para as cor branca da linha e criando uma mascara
        hsv1 = np.array([0 , 0, 220], dtype=np.uint8)
        hsv2 = np.array([180 , 50, 255], dtype=np.uint8)
        mask = cv2.inRange(img_hsv, hsv1, hsv2)
        
        #Fazendo um blur para diminuir o ruido e transformando em binario + eliminando 'buracos'
        mask_blur = cv2.blur(mask, (3,3))
        mask = cv2.morphologyEx(mask_blur,cv2.MORPH_CLOSE,np.ones((10, 10)))
        
        #Definindo os contornos
        contornos, arvore = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        #aplicando autocanny para achar as bordas
        bordas = auto_canny(mask_blur.copy())
        cdst = cv2.cvtColor(bordas, cv2.COLOR_GRAY2BGR)
        
        '''
        #aplicando canny e blur
        bgr_blur = cv2.blur(img_bgr, (3,3)) #aplicando blur para melhorar detecacao de bordas
        bordas = auto_canny(bgr_blur.copy())
        cdst = cv2.cvtColor(bordas, cv2.COLOR_GRAY2BGR) # Converte a imagem para BGR para permitir desenho colorido
        '''
        if True: # HoughLinesP
            lines = cv2.HoughLinesP(bordas, 10, math.pi/180.0, 100, np.array([]), 5, 5)
            print("Used Probabilistic Rough Transform")
            print("The probabilistic hough transform returns the end points of the detected lines")
            a,b,c = lines.shape
            print("Valor de A",a, "valor de lines.shape", lines.shape)
            
            linha_e = False
            linha_d = False
            for i in range(a):
                x1 = lines[i][0][0]
                y1 = lines[i][0][1]
                x2 = lines[i][0][2]
                y2 = lines[i][0][3]
                tam = (((y2-y1)**2)+((x2-x1)**2))**0.5
                maior = 80
                if tam>maior:
                    m = (y2-y1)/(x2-x1) #coef angular da reta
                    if linha_e==False:
                        if (m>-4 and m<-0.05):
                            me = m
                            xe1 = x1
                            ye1 = y1
                            xe2 = x2
                            ye2 = y2
                            linha_e = True
                            # Faz uma linha ligando o ponto inicial ao ponto final, com a cor vermelha (BGR)
                            cv2.line(img_bgr, (lines[i][0][0], lines[i][0][1]), (lines[i][0][2], lines[i][0][3]), (0, 0, 255), 3, cv2.LINE_AA)
                    if linha_d==False:
                        if (m>0.05 and m<4):
                            md = m
                            xd1 = x1
                            yd1 = y1
                            xd2 = x2
                            yd2 = y2
                            linha_d = True
                            # Faz uma linha ligando o ponto inicial ao ponto final, com a cor verde (BGR)
                            cv2.line(img_bgr, (lines[i][0][0], lines[i][0][1]), (lines[i][0][2], lines[i][0][3]), (0, 255, 0), 3, cv2.LINE_AA)
            
            #se eu tiver tanto a linha esq quanto a dir --> desenha o ponto
            if linha_e and linha_d:
                xi = ((yd1-md*xd1)-(ye1-me*xe1))/(me-md)
                yi = me*(xi-xe1)+ye1
                cv2.circle(img_bgr,(int(xi),int(yi)),2,(255,0,0),3)
                

        else:    # HoughLines
            # Esperemos nao cair neste caso
            lines = cv2.HoughLines(bordas, 1, math.pi/180.0, 50, np.array([]), 0, 0)
            a,b,c = lines.shape
            for i in range(a):
                rho = lines[i][0][0]
                theta = lines[i][0][1]
                a = math.cos(theta)
                b = math.sin(theta)
                x0, y0 = a*rho, b*rho
                pt1 = ( int(x0+1000*(-b)), int(y0+1000*(a)) )
                pt2 = ( int(x0-1000*(-b)), int(y0-1000*(a)) )
                cv2.line(img_bgr, pt1, pt2, (0, 0, 255), 3, cv2.LINE_AA)
            print("Used old vanilla Hough transform")
            print("Returned points will be radius and angles")
        
        
            
        #Mostrando o resultado
        #cv2.imshow("source", src)
        cv2.imshow("detected", img_bgr)
        cv2.imshow("detected lines", cdst)
        cv2.imshow("detected _lines", mask)
        
        
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()


