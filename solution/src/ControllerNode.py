#!/usr/bin/env python
#Importacion de librerias y topicos para nuestro codigo
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from math import pi
from std_msgs.msg import String
import numpy as np

class square:
        def __init__(self):

            #declaracion de variables utilizadas(radio,longitud, angulo, posicion x, etc)
            self.wr = 0.00
            self.wl = 0.00
            self.r = 0.05
            self.l = 0.18

            self.P_X = 0.00
            self.P_Y = 0.00
            self.angle = 0.00
            #Declaracion de variables de posicion target y valores propocionales del controlador 
            self.XTarget = 1
            self.YTarget = 1

            self.KV = 0.1
            self.KW = 0.27

            #Nos suscribimos a los topics de  wheel right y wheel left
            rospy.Subscriber('/wr', Float32, self.wr_callback)
            rospy.Subscriber('/wl', Float32, self.wl_callback)
            #Creacion de publisher para topicos de odometry, cmd vel y chatter
            self.cmd_pub = rospy.Publisher('/cmd_vel', Twist,queue_size=10)
            self.odom_pub = rospy.Publisher('/odometry', Pose2D,queue_size=10)
            self.chat_pub = rospy.Publisher('/chatter', String,queue_size=10)
            #Creacion de nodo
            rospy.init_node("ControllerNode")
            self.rate = rospy.Rate(10)
            #En caso de shot down llamar funcion stop
            rospy.on_shutdown(self.stop)

        #Callbacks para cambiar valores de wheel right and left
        def wr_callback(self,msg):
            self.wr = msg.data

        def wl_callback(self,msg):
            self.wl = msg.data
        #Codio principal
        def run(self):
            #Declaracion de valores de tiempo para conseguir dt
            current_time = rospy.get_time()
            last_time = rospy.get_time()

            #Inicializacion de valores para geometry twist
            msg = Twist()
            msg.linear.x = 0
            msg.linear.y = 0
            msg.linear.z = 0
            msg.angular.x = 0
            msg.angular.y = 0
            msg.angular.z = 0
            #Incializacion de valores para odometria 
            odometry = Pose2D()
            odometry.theta = self.angle
            odometry.x = self.P_X
            odometry.y = self.P_Y
            #Loop principal
            while not rospy.is_shutdown():
                #Conseguir valor de dt con la diferencia de tiempos
                current_time = rospy.get_time()
                dt = current_time - last_time
                last_time = current_time
                #Calculo de valores de odometria(Posicion Y, Posicion X y Angulo)
                self.angle = self.angle + self.r*((self.wr-self.wl)/self.l)* dt
                self.P_X = self.P_X + (self.r*((self.wr+self.wl)/2)* dt*np.cos(self.angle))
                self.P_Y = self.P_Y + (self.r*((self.wr+self.wl)/2)* dt*np.sin(self.angle))
                #Caluclo de error angulo y distancia
                self.etheta = np.arctan2(self.XTarget,self.YTarget) - self.angle
                self.ed = np.sqrt((self.XTarget - self.P_X)**2+(self.YTarget - self.P_Y)**2)
                #Asignacion de valores de odometria
                odometry.theta = self.angle
                odometry.x = self.P_X
                odometry.y = self.P_Y
                
                self.error = str(self.ed)
                self.arctan =  np.arctan2(self.XTarget,self.YTarget)
                #Calculos de v y w para nuestro controlador proporcional
                self.v = self.KV*self.ed
                self.w = self.KW*self.etheta              
                #Si nuestro error es menor a 0.07 se detiene nuestro puzzlebot
                if self.etheta < 0.07 and self.ed < 0.07:
                    self.v = 0
                    self.w = 0  
                    self.stop()

                #Control de valores propocionales dependiendo del error
                if self.P_X/self.P_Y > 1.6:
                    if self.P_X > self.P_Y:
                        self.w = self.w + 0.1
                    if self.P_X < self.P_Y:
                        self.v = self.v + 0.1

                if self.P_Y/self.P_X > 1.6:
                    if self.P_X > self.P_Y:
                        self.w = self.w + 0.1
                    if self.P_X < self.P_Y:
                        self.v = self.v + 0.1
                    
                #Imprimir posicion y errores
                print("Angle: " + str(self.angle))

                print("Error: " + self.error + " Error Angulo: " + str(self.etheta))
                print("X: "  + str(self.P_X) + " Y: " + str(self.P_Y))
                #Asignacion de valores v y w para publicar en cmd vel
                msg.linear.x  = self.v
                msg.angular.z = self.w
                #Publicacion de valores en topico
                self.chat_pub.publish(self.error)
                self.cmd_pub.publish(msg)
                self.odom_pub.publish(odometry)
                self.rate.sleep()

        #Funcion de detencion de puzzlebot
        def stop(self):
            print("Stopping")
            msg = Twist()
            msg.linear.x = 0
            msg.linear.y = 0
            msg.linear.z = 0
            msg.angular.x = 0
            msg.angular.y = 0
            msg.angular.z = 0
            self.w_pub.publish(msg)

if __name__ == "__main__":
    #Inicializacion de clase y ejecucion de codigo
    sq = square()
    try:
        sq.run()
    except rospy.ROSInterruptException:
        None