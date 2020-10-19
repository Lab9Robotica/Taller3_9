import cv2
import numpy as np
import argparse
parser = argparse.ArgumentParser()
parser.add_argument("video", help="Ingrese el nombre del video")
args=parser.parse_args()
archivo_video=args.video
capture=cv2.VideoCapture(archivo_video) #Se lee el video pasado por parámetro


class Jugador():
    def __init__(self,coordenadas,numero,verdes,fucsias):
        self.coordenadas=coordenadas
        self.verdes=verdes
        self.fucsias=fucsias
        self.numero=numero
    def darVerdes(self):
        return self.verdes
    def darFucsias(self):
        return self.fucsias
    def darNumero(self):
        return self.numero
    def darPos(self):
        return self.coordenadas 
    def cambiarNumero(self):
        todos=self.verdes+self.fucsias
        if len(self.verdes)==3:
            self.numero=2
        elif len(self.fucsias)==3:
            self.numero=0
        elif pitagorazo(self.verdes[0][0],self.verdes[1][0],self.verdes[0][1],self.verdes[1][1])>17:
            self.numero=3
        else:
            self.numero=1
            pass






def pitagorazo(x1,x2,y1,y2):
    distancia=np.sqrt((x2-x1)**2+(y2-y1)**2)
    return distancia
def rescale_frame(frame, percent=75):
    width = int(frame.shape[1] * percent/ 100)
    height = int(frame.shape[0] * percent/ 100)
    dim = (width, height)
    return cv2.resize(frame, dim, interpolation =cv2.INTER_AREA)
def tuplaFtoI(tupla):
    nuevaTupla=[]
    
    for i in tupla:
        nuevaTupla.append(int(i))
    nuevaTupla=tuple(nuevaTupla)
    
    return nuevaTupla
        
def sacarCentros(maskE):
     #Para azules
    contours,_ = cv2.findContours (maskE, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) # cv2.findContours(image, mode, method[, contours[, hierarchy[, offset]]]) -> contours, hierarchy		 
    centros = []
    contador=0
    for contour in contours:
        centros.append (cv2.minEnclosingCircle (contour)[0])
    return centros
        
def identificar(centro,distancia,colorBuscado):
    cercanos=[]
    posibles=sacarCentros(colorBuscado)
    for i in posibles:
        distanciaEu=pitagorazo(i[1],centro[1],i[0],centro[0])    
        if distanciaEu<distancia:
            cercanos.append(i)
            #print('La posición encontrada es:')
            #print(i)
            #print('En el centro:')
            #print(centro)
    return cercanos



circles=np.zeros((4,2),np.int)        
counter=0
def mousePoints(event,x,y,flags,params):
    global counter
    if event == cv2.EVENT_LBUTTONDOWN:
        
        circles[counter]=x,y
        counter+=1
        print(circles)

    #cv2.circle(cap,(x,y),20,(255,255,255),2)
    
cap=cv2.VideoCapture(archivo_video)  

rval, im_src = cap.read()
cap80 = rescale_frame(im_src, percent=80)
cap.release()

while True:
    
    if counter==4:
        puntos1=np.float32(circles)*5/4
        
        puntos2=np.float32([[0,0],[900,0],[0,600],[900,600]])
        matrix=cv2.getPerspectiveTransform(puntos1,puntos2)
        result=cv2.warpPerspective(im_src,matrix,(900,600))
        cv2.imshow('Imagen2',result)
        
        
    
    
    cv2.imshow('Imagen',cap80)
    cv2.setMouseCallback('Imagen',mousePoints)
    k = cv2.waitKey(1) & 0xFF
    if k==27:
        break
        
cv2.destroyAllWindows()


    

    
while True:
    
    rect, frame=capture.read()
    
    
    #Se reescala la imagen al 90% debido a que la alta resolución del video queda cortado
    frame80 = rescale_frame(frame, percent=90)
    #cv2.setMouseCallBack(frame80,mousePoints) 
    #Se aplica la nueva perspectiva basada en las 4 esquinas  a una resolución de 900x600
    puntos1=np.float32(circles)*5/4
    puntos2=np.float32([[0,0],[900,0],[0,600],[900,600]])
    matrix=cv2.getPerspectiveTransform(puntos1,puntos2)
    result=cv2.warpPerspective(frame,matrix,(900,600))
    cv2.imshow('Imagen2',result)  
    
    
      
    result=cv2.warpPerspective(frame,matrix,(900,600))
    
    #Pasar video a HSV para aplicar filtros
    hsv=cv2.cvtColor(result,cv2.COLOR_BGR2HSV)
    
    
    #Para fucsia [160  26  60]
    #Para Azul [120 255 254]
    #Para Verde [ 65 255 255]
    #Para negro [0 0 0]
    #Para verde cancha [ 61 167  67]
    #Para amarillo [ 30 255 255]
    
    
    #Se aplican los filtros para los colores que identifican a los jugadores
    #Rangos para fucsia
    low_fucsia=np.array([130,200,200])
    up_fucsia=np.array([200,255,255])
    #Rangos para azul
    low_azul=np.array([105,200,200])
    up_azul=np.array([130,255,255])
    #Rangos para verde
    low_verde=np.array([50,200,200])
    up_verde=np.array([65,255,255])
    #Rangos para amarillo
    low_amarillo=np.array([25,200,200])
    up_amarillo=np.array([35,255,255])
    #Rangos para negro
    low_negro=np.array([0,0,0])
    up_negro=np.array([80,80,80])
    #Rangos para cancha verde
    low_cancha=np.array([60,160,60])
    up_cancha=np.array([65,180,70])

    #Rangos para blanco
    low_blanco=np.array([215,215,215])
    up_blanco=np.array([255,255,255])
    
    #Máscaras para cada color 
    mask_fucsia=cv2.inRange(hsv,low_fucsia,up_fucsia)
    mask_azul=cv2.inRange(hsv,low_azul,up_azul)
    mask_verde=cv2.inRange(hsv,low_verde,up_verde)
    mask_amarillo=cv2.inRange(hsv,low_amarillo,up_amarillo)
    mask_negro=cv2.inRange(hsv,low_negro,up_negro)
    mask_cancha=cv2.inRange(hsv,low_cancha,up_cancha)
    mask_blanco=cv2.inRange(hsv,low_blanco,up_blanco)
    
    res1=cv2.bitwise_and(result,result,mask=mask_fucsia)
    res2=cv2.bitwise_and(result,result,mask=mask_azul)
    res3=cv2.bitwise_and(result,result,mask=mask_verde)
    res4=cv2.bitwise_and(result,result,mask=mask_amarillo)
    res5=cv2.bitwise_and(result,result,mask=mask_negro)
    
    kernel = np.ones((5,5), np.uint8) 
    #Se aplican erode() y dilate()
    
    e_fucsia=cv2.erode(mask_fucsia, kernel, iterations=1)
    d_fucsia=cv2.dilate(mask_fucsia, kernel, iterations=1)
    
    e_azul=cv2.erode(mask_azul, kernel, iterations=1)
    d_azul=cv2.dilate(mask_azul, kernel, iterations=1)
    
    e_verde=cv2.erode(mask_verde, kernel, iterations=1)
    d_verde=cv2.dilate(mask_verde, kernel, iterations=1)
    
    e_amarillo=cv2.erode(mask_amarillo, kernel, iterations=1)
    d_amarillo=cv2.dilate(mask_amarillo, kernel, iterations=1)
    
    e_negro=cv2.erode(mask_negro, kernel, iterations=1)
    d_negro=cv2.dilate(mask_negro, kernel, iterations=1)
    
    
    #Aplicando cv2.findContours y cv2.minEnclosingCircle
	#Hallar centros de jugadores
    
    centros_azules=sacarCentros(d_azul)
    centros_amarillos=sacarCentros(d_amarillo)
    
    #Hallar los colores cercanos a cada jugador
    conjunto_jugadoresA=[]
    contador=0
    for j in centros_azules:
        jugadorActual=Jugador(j,contador,identificar(j,15,d_verde),identificar(j,15,d_fucsia))
        conjunto_jugadoresA.append(jugadorActual)
        contador+=1
        
    
    conjunto_jugadoresY=[]
    contador=0
    for j in centros_amarillos:
        jugadorActual=Jugador(j,contador,identificar(j,15,d_verde),identificar(j,15,d_fucsia))
        conjunto_jugadoresY.append(jugadorActual)
        contador+=1
    #print(conjunto_jugadoresA[2].darVerdes())
   
    #cv2.circle(result, tuplaFtoI(conjunto_jugadoresA[3].darFucsias()[2]), 5,(0,0,255),-1)
    #cv2.circle(result, tuplaFtoI(conjunto_jugadoresA[3].darFucsias()[1]), 5,(0,0,255),-1)
    ambos=zip(conjunto_jugadoresA,conjunto_jugadoresY)
    font = cv2.FONT_HERSHEY_SIMPLEX
    for j,i in ambos:
        i.cambiarNumero()
        j.cambiarNumero()
        cv2.putText(result, str(j.darNumero()),tuplaFtoI(j.darPos()) ,font,1,(0,255,0),1,cv2.LINE_AA)
        cv2.putText(result, str(i.darNumero()),tuplaFtoI(i.darPos()) ,font,1,(0,255,0),1,cv2.LINE_AA)
        
        
    #print(conjunto_jugadoresA[3].darNumero())
    #cv2.putText(result, str(conjunto_jugadoresA[3].darNumero()),tuplaFtoI(conjunto_jugadoresA[3].darPos()) ,font,1,(0,255,0),1,cv2.LINE_AA)
    #cv2.putText(image, 'OpenCV', org, font,  fontScale, color, thickness, cv2.LINE_AA) 
                   
                   
    
    
   
   

   
     
    #Encontramos los circulos cercanos al centro de cada jugador
    #matriz_esquinas=np.zeros((len(center_azules,4)))
    #for i in center_azules:
       # i[0]+=15
    
    identity=d_fucsia+d_azul+d_verde+d_amarillo

    cv2.imshow('frame80', frame80)
    cv2.imshow('Perspectiva nueva',result)
    #cv2.imshow('todos',identity)
    #cv2.imshow('fucsia',d_fucsia)
    #cv2.imshow('azul',d_azul)
    #cv2.imshow('verde',d_verde)
    #cv2.imshow('amarillo',d_amarillo)
    #cv2.imshow('negro',d_negro)
    #cv2.imshow('hull',hull)
    #print (radius)
    #print(center)
    
    
    
    
    
    
    
    
    key=cv2.waitKey(1)
    if key==27: #Presionar tecla S para terminar. 
        break

capture.release()
cv2.destroyAllWindows()