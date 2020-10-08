#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Float32MultiArray
import matplotlib.pyplot as plt
from matplotlib import animation
import threading

posSIMx = []
posSIMy = []

global posBall
posBall = []

def callbackPos (msg):
	posSIMx.append(msg.linear.x)
	posSIMy.append(msg.linear.y)


def callbackBallPos (msg):
	global posBall
	posBall = msg.data


def graficar(a):
	plt.cla()	# Se borra informacion anterior al ploteo actual
	plt.plot( posSIMx, posSIMy )	# Plotea las variables
	plt.axis([ -6.6, 6.6, -5, 5 ])	#Define limites en X y Y (limites del cancha)


def grafica():
	objeto = animation.FuncAnimation(plt.figure(1), graficar, 10000)
	plt.show()


def soccer_player():
	# Inicializa el nodo.
	rospy.init_node('soccer_futbol_player', anonymous=True)

	#Inicializa los Subscribers
	rospy.Subscriber('robot_Position', Twist, callbackPos)
	rospy.Subscriber('ball_Position', Float32MultiArray, callbackBallPos)

	# Inicializa los Publishers
	pubKick = rospy.Publisher('kick_power', Float32, queue_size=10)
	pubVel = rospy.Publisher('robot_move_vel', Twist, queue_size=10)

	# Crea el hilo para la grafica en tiempo real
	graph = threading.Thread(target=grafica)
	graph.start()

	rate = rospy.Rate(10)	#10Hz

	while not rospy.is_shutdown():

		#Inicializa los mensajes
		msgVel = Twist()
		msgKick = Float32()

		# Asigna datos a los mensajes
		msgVel.linear.x =	0.1	# TODO: Asignar valores.
		msgVel.linear.y = 0	# TODO: Asignar valores.
		msgVel.angular.z = 0	# TODO: Asignar valores.

		msgKick.data = 0 		# TODO: Asignar valores.

		# Publica los mensajes en el topic respectivo.
		pubVel.publish(msgVel)
		pubKick.publish(msgKick)

		rate.sleep()


if __name__ == '__main__':
	try:
		soccer_player()
	except rospy.ROSInterruptException:
		pass