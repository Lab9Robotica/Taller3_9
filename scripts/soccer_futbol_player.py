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
global rho, beta, posFin, thetaAct, velDir, nextStep, ball

# Coordenadas para la transformacion
rho = 0.1
beta = 0

# Posicion a la que se quiere llegar
posFin = [ 0, 0 ]

# Orientacion del robot
thetaAct = 0.15

# Direccion de la velocidad para obtener sus componentes
velDir = 0

# Constantes del controlador de velocidad
kp = 0.4
kbeta = 0.5

# Posicion del robot para la grafica
posSIMx = []
posSIMy = []

# Posicion de la pelota
ball = []

# Booleano para el paso puntos
nextStep = False


def callbackPos (msg):
	global thetaAct
	posSIMx.append(msg.linear.x)
	posSIMy.append(msg.linear.y)

	thetaAct = msg.angular.z


def posicionFinal(posBall):
	global nextStep

	# Eleccion de la cancha mas cercana
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

	dXc = x - posBall[0]
	dXr = x - posSIMx[-1]

	# Este ciclo determina si el robot esta mas cerca a la cancha que la pelota
	if abs(dXc)>abs(dXr):
		nextStep = False
		if posBall[1]>0:
			y = posBall[1] - 0.5
		else:
			y = posBall[1] + 0.5

		if posBall[0] > 0:
			x = posBall[0] - 0.1
		else:
			x = posBall[0] + 0.1

		dX = x - posSIMx[-1]
		dY = y - posSIMx[-1]
		theta = math.atan2( dY, dX )

		return [ x, y ], theta

	# Determina el punto en el cual el robot debe ir antes de acercarse a la pelota
	if not nextStep:
		dX = x - posBall[0]

		theta = math.atan2( -posBall[1], dX )

		x = posBall[0] - 0.2*np.cos(theta)
		y = posBall[1] - 0.2*np.sin(theta)

		if abs(rho)>0.03:
			return [ x, y ], theta
		else:
			nextStep = True

	# Determina el punto final
	dX = x - posBall[0]

	theta = math.atan2( -posBall[1], dX )

	x = posBall[0] - 0.09*np.cos(theta)
	y = posBall[1] - 0.09*np.sin(theta)

	return [x,y], theta


def callbackBallPos (msg):
	global rho, beta, posFin, velDir, ball

	posFin, beta = posicionFinal(msg.data)
	ball = msg.data

	# Diferencial de la posicion
	dX = posFin[0] - posSIMx[-1]
	dY = posFin[1] - posSIMy[-1]

	# Transformacion de coordenadas
	rho = np.sqrt( dX**2 + dY**2 )

	# En los dos ultimos puntos el robot ya esta alineado mirando hacia la cancha
	if nextStep:
		velDir = beta
	else:
		velDir = math.atan2( dY, dX )


def graficar(a):
	plt.cla()	# Se borra informacion anterior al ploteo actual
	plt.title('Trayectoria del robot')
	plt.xlabel('X')
	plt.ylabel('Y')
	plt.plot( posSIMx, posSIMy )	# Plotea las variables
	plt.scatter(posFin[0],posFin[1])
	plt.scatter(ball[0], ball[1], marker="x", c="r")
	plt.axis([ -6.6, 6.6, -5, 5 ])	#Define limites en X y Y (limites de la cancha)
	


def grafica():
	objeto = animation.FuncAnimation(plt.figure(1), graficar, 10000)
	plt.show()	


def soccer_player():
	global rho, beta, nextStep, llego, listo

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

		# Alinea al robot mirando hacia posicion dada por la funcion posicionFinal
		if abs(velDir - thetaAct) > 0.08:
			w = kbeta*(velDir - thetaAct)

			msgVel.linear.x = 0
			msgVel.linear.y = 0
			msgVel.angular.z = w
			pubVel.publish(msgVel)
		# Lleva al robot hasta la posicion dada por la funcion posicionFinal
		else:
			v = kp*rho
			msgVel.linear.x =	v*np.sin( velDir - thetaAct )
			msgVel.linear.y = v*np.cos( velDir - thetaAct )
			msgVel.angular.z = 0
			if abs(rho)>0.03:
				pubVel.publish(msgVel)
			else:
				msgVel.linear.x = 0
				msgVel.linear.y = 0
				pubVel.publish(msgVel)

		rate.sleep()


if __name__ == '__main__':
	try:
		soccer_player()
	except rospy.ROSInterruptException:
		pass