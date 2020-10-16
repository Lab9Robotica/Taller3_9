#!/usr/bin/env python3	

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Float32MultiArray
import matplotlib.pyplot as plt
from matplotlib import animation
from threading import Thread
import numpy as np
import math

# Variables globales
global rho, alpha, beta, posFin, thetaAct, velDir, alineado, llego, listo

# Coordenadas para la transformacion
rho = 0.1
alpha = 0.2
beta = 0

# Posicion a la que se quiere llegar
posFin = [ 0, 0 ]

# Orientacion del robot
thetaAct = 0.15

# Direccion de la velocidad para obtener sus componentes
velDir = 0

# Indican si el robot ya se alineó mirando hacia la cancha, si llego a la pelota y si
# el reccorrido termino
alineado = False
llego = False
listo = False

# Constantes del controlador de velocidad
kp = 0.33
kalpha = 0.1
kbeta = 0.5

# Posicion del robot para la grafica
posSIMx = []
posSIMy = []


def callbackPos (msg):
	global thetaAct
	posSIMx.append(msg.linear.x)
	posSIMy.append(msg.linear.y)

	thetaAct = msg.angular.z


def posicionFinal(posBall):
	x = 0
	if len(posSIMx)>0:
		if posBall[0]>0:
			x = 6
		elif posBall[0]<0:
			x = -6
		else:
			if posSIMx[-1] > 0:
				x = -6
			else:
				x = 6

	dX = x - posBall[0]

	theta = math.atan2( -posBall[1], dX )

	x = posBall[0] - 0.09*np.cos(theta)
	y = posBall[1] - 0.09*np.sin(theta)

	return [ x, y ], theta


def callbackBallPos (msg):
	global rho, alpha, beta, posFin, velDir

	posFin, beta = posicionFinal(msg.data)

	# Diferencial de la posicion
	dX = posFin[0] - posSIMx[-1]
	dY = posFin[1] - posSIMy[-1]

	# Transformacion de coordenadas
	rho = np.sqrt( dX**2 + dY**2 )
	velDir = math.atan2( dY, dX )


def graficar(a):
	plt.cla()	# Se borra informacion anterior al ploteo actual
	plt.plot( posSIMx, posSIMy )	# Plotea las variables
	plt.axis([ -6.6, 6.6, -5, 5 ])	#Define limites en X y Y (limites de la cancha)


def grafica():
	objeto = animation.FuncAnimation(plt.figure(1), graficar, 10000)
	plt.show()


def soccer_player():
	global rho, alpha,beta, alineado, llego, listo

	# Inicializa el nodo.
	rospy.init_node('soccer_futbol_player', anonymous=True)

	#Inicializa los Subscribers
	rospy.Subscriber('robot_Position', Twist, callbackPos)
	rospy.Subscriber('ball_Position', Float32MultiArray, callbackBallPos)

	# Crea el thread para la grafica en tiempo real
	graph = Thread(target=grafica)
	graph.start()

	# Inicializa el publicador de la velocidad
	pubVel = rospy.Publisher('robot_move_vel', Twist, queue_size=10)

	rate = rospy.Rate(10)	#10Hz

	#Inicializa el mensaje
	msgVel = Twist()

	while not rospy.is_shutdown():

		# Alinea al robot mirando hacia la cancha
		while abs(beta - thetaAct) > 0.01 and not listo:
			w = kbeta*(beta - thetaAct)
			msgVel.angular.z = w
			pubVel.publish(msgVel)

			if abs(beta - thetaAct) < 0.01:
				listo = True
				msgVel.angular.z = 0
				pubVel.publish(msgVel)

		# Acerca al robot hasta la pelota
		while abs(rho) > 0.03 and not llego:
			v = kp*rho
			msgVel.linear.x =	v*np.sin( velDir - thetaAct )
			msgVel.linear.y = v*np.cos( velDir - thetaAct )
			pubVel.publish(msgVel)
			if abs(rho)<0.03:
				llego = True
				msgVel.linear.x = 0
				msgVel.linear.y = 0
				pubVel.publish(msgVel)
		rate.sleep()

		print('Hecho')


if __name__ == '__main__':
	try:
		soccer_player()
	except rospy.ROSInterruptException:
		pass