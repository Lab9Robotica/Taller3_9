import numpy as np
import matplotlib.pyplot as plt
import cv2
import argparse

#Obtener el video
def firstFunc():
    ruta='/home/jsgonzalez15/robotica_ws/src/taller3_9/resource/'
    parse=argparse.ArgumentParser()
    parse.add_argument('nombre',help='Nombre del archivo .mp4 a transformar',type=str)
    args=parse.parse_args()
    nombre=args.nombre
    video1 = cv2.VideoCapture(ruta+nombre+'.mp4')

    while(video1.isOpened()):
        ret, frame = video1.read() # Recorrido frame por frame
        if ret == True:
            cv2.imshow('Frame',frame) # Mostrar frame actual
            if cv2.waitKey(25) & 0xFF == ord('q'): # Press Q on keyboard to exit
                break 
        else: 
            break
        
    video1.release() # Paso final para cerrar el video
    cv2.destroyAllWindows() # Cierra todos los frames

if __name__ == "__main__":
    firstFunc()
