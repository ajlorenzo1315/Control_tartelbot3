#!/usr/bin/env python

import os
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped

from nav_msgs.msg import Odometry
from std_msgs.msg import String

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# https://answers.ros.org/question/207110/there-is-no-move_base-node-in-move_base-package/
# si se tiene problemas
# http://wiki.ros.org/amcl
#Can you see move base in the list when you run ls /opt/ros/indigo/lib/move_base/ 

#If not, this means that move base has not been properly installed. So try: sudo apt-get install ros-"version de ros"-move-base
# sudo apt-get install ros-noetic-move-base
# sudo apt-get install ros-noetic-map-server
# https://github.com/YI-LING-BA-JI-JHUAN-TI-SHENG/documents/blob/master/problems/p1.md

# NOTE: this example requires PyAudio because it uses the Microphone class
#https://stackoverflow.com/questions/14257598/what-are-language-codes-in-chromes-implementation-of-the-html5-speech-recogniti
#https://stackoverflow.com/questions/7088672/pyaudio-working-but-spits-out-error-messages-each-time
# Para reconocer la voz entrada
import speech_recognition as sr
# sonido del asistende virtual
# pip install gTTS
from gtts import gTTS

# control por gestos
import cv2
import mediapipe as mp
import time
from collections import deque

import numpy as np
import math

import re
from unicodedata import normalize
from gtts import gTTS


# para leer la memoria del robot
import csv

# From alsa-lib Git 3fd4ab9be0db7c7430ebd258f2717a976381715d
# $ grep -rn snd_lib_error_handler_t
# include/error.h:59:typedef void (*snd_lib_error_handler_t)(const char *file, int line, const char *function, int err, const char *fmt, ...) /* __attribute__ ((format (printf, 5, 6))) */;
# Define our error handler type

from ctypes import *
from contextlib import contextmanager
import pyaudio


ERROR_HANDLER_FUNC = CFUNCTYPE(None, c_char_p, c_int, c_char_p, c_int, c_char_p)

def py_error_handler(filename, line, function, err, fmt):
    pass

c_error_handler = ERROR_HANDLER_FUNC(py_error_handler)

@contextmanager
def noalsaerr():
    asound = cdll.LoadLibrary('libasound.so')
    asound.snd_lib_error_set_handler(c_error_handler)
    yield
    asound.snd_lib_error_set_handler(None)

c_error_handler = ERROR_HANDLER_FUNC(py_error_handler)

with noalsaerr():
    p = pyaudio.PyAudio()
stream = p.open(format=pyaudio.paFloat32, channels=1, rate=44100, output=1)


class ubicacion_data:
    def __init__(self,dic):
        #ubicacion,pos_x,pos_y,pos_z,orientacion_x,orientacion_y,orientacion_z,orientacion_w
        self.ubicacion=dic["ubicacion"]
        self.px=float(dic["pos_x"])
        self.py=float(dic["pos_y"])
        self.pz=float(dic["pos_z"])
        
        self.ox=float(dic["orientacion_x"])
        self.oy=float(dic["orientacion_y"])
        self.oz=float(dic["orientacion_z"])
        self.ow=float(dic["orientacion_w"])
        self.keys_guar=list(dic.keys())
    

    def __repr__(self):
        return " , ".join([str(self.px),str(self.pz),str(self.py)])
        
class agente:

    def __init__(self,memoria_path=None,path=None):

    
        self.r = sr.Recognizer()
        self.m=sr.Microphone() 

        rospy.init_node('goal_pose')
        self.cuadrar_posicion_mapa_real()
        # Get initial pose from Gazebo
        odom_msg = rospy.wait_for_message('/odom', Odometry)
        dic_inicio={}
        dic_inicio["ubicacion"]="inicio"
        dic_inicio["pos_x"]=odom_msg.pose.pose.position.x
        dic_inicio["pos_y"]=odom_msg.pose.pose.position.y
        dic_inicio["pos_z"]=odom_msg.pose.pose.position.z

        dic_inicio["orientacion_x"]=odom_msg.pose.pose.orientation.x
        dic_inicio["orientacion_y"]=odom_msg.pose.pose.orientation.y
        dic_inicio["orientacion_z"]=odom_msg.pose.pose.orientation.z
        dic_inicio["orientacion_w"]=odom_msg.pose.pose.orientation.w

        self.ubicaciones={}
        self.ubicaciones[dic_inicio["ubicacion"]]=ubicacion_data(dic_inicio)
        self.keys_save_ubicacion=list(dic_inicio.keys())

        if memoria_path:
            with open(memoria_path, newline='') as csvfile:
                reader = csv.DictReader(csvfile)
                for row in reader:
                    self.ubicaciones[row["ubicacion"]]=ubicacion_data(row)
            self.memoria_path=memoria_path
        else:
            memoria_path=os.path.realpath(__file__).split("/")
            memoria_path[-1]='memoria/ubicaiones.csv'
            memoria_path="/".join(memoria_path)

        self.msg_goal=MoveBaseGoal()
        self.navclient=actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.navclient.wait_for_server()

        #rospy.init_node('movedor_robot')
        self.pub_vel = rospy.Publisher('/cmd_vel',Twist,queue_size=1) # Movimiento del turtlebot3
        self.scan = rospy.Subscriber('/scan',LaserScan,self.scan_callback) # Escaner del turtlebot3
        
        self.rate = rospy.Rate(5) # 10
        
        self.move_msg = Twist()
        self.min_dist=100
        self.dist_seguridad=0.5

        self.memoria_rutas={}
        if path:

            self.get_info_ruta_csv(path)

        else:
            path=os.path.realpath(__file__).split("/")
            path[-1]='memoria/ruta.csv'
            path="/".join(path)

        self.memoria_path_ruta=path

        self.keys_save_ruta='ubicacion,pos_x,pos_y,pos_z,orientacion_x,orientacion_y,orientacion_z,orientacion_w,ruta,espera'.split(',')

        self.ejecutando_accion_ros=0
        self.ruta_posicion_alcanzada=0

    @staticmethod
    def transformar_str_rut_to_rut(ruta_str):
         return [list(map(float,vels.split("_"))) for vels in ruta_str.split('p')]

    def get_info_ruta_csv(self,path):

        with open(path, newline='') as csvfile:
                reader = csv.DictReader(csvfile)
                for row in reader:
                    ruta_str=row["ruta"]
                    self.memoria_rutas[row["ubicacion"]]={"inicio":ubicacion_data(row),"ruta":self.transformar_str_rut_to_rut(ruta_str)\
                    ,"ruta_str":ruta_str,"espera":list(map(float,row["espera"].split('_')))}
                    #self.ubicaciones[row["ubicacion"]]=ubicacion_data(row)
                

    def cuadrar_posicion_mapa_real(self):

        pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)

        # Construct message
        init_msg = PoseWithCovarianceStamped()
        init_msg.header.frame_id = "map"

        # Get initial pose from Gazebo
        odom_msg = rospy.wait_for_message('/odom', Odometry)
        init_msg.pose.pose.position.x = odom_msg.pose.pose.position.x
        init_msg.pose.pose.position.y = odom_msg.pose.pose.position.y
        init_msg.pose.pose.orientation.x = odom_msg.pose.pose.orientation.x
        init_msg.pose.pose.orientation.y = odom_msg.pose.pose.orientation.y
        init_msg.pose.pose.orientation.z = odom_msg.pose.pose.orientation.z
        init_msg.pose.pose.orientation.w = odom_msg.pose.pose.orientation.w

        print("Posicion: ", init_msg.pose.pose.position.x, init_msg.pose.pose.position.y)
        print("Orientacion: ", init_msg.pose.pose.orientation.x, init_msg.pose.pose.orientation.y, init_msg.pose.pose.orientation.z, init_msg.pose.pose.orientation.w)

        # Delay
        rospy.sleep(1)

        # Publish message
        rospy.loginfo("setting initial pose")
        pub.publish(init_msg)
        rospy.loginfo("initial pose is set")


    def scan_callback(self,msg):

        """
        Toma los valores del escaner. Definimos las regiones en un diccionario en lugar de definirlas, 
        porque los necesitamos en el mismo lugar, para decidir en que estado estamos
        """
        
        self.regions = {
            # 360 ranges
            'right': np.min(msg.ranges[270:271]), # 270
            'front_right': np.min(msg.ranges[315:320]), # 315
            'front': np.min(msg.ranges[0:1]), # 480:719
            'front_left': msg.ranges[45], # 720:959
            'left': msg.ranges[90], # 960:1199
            'backwards': msg.ranges[180]
        }

        self.min_dist=np.min(msg.ranges)


    def change_state(self,state):
        """
        Cambia al estado que encaja mejor en cada caso 
        """
        
        if state is not self.state1: # If the actual state is different from the previous one
            print('\nWall follower - [state %s] - %s' % (state, self.state_dict_[state])) # Says in which state we are (line 21)
            self.state1 = state


    def lets_go(self):

        left_distance = 0.75 # 1.0
        right_distance = 0.75 # 1.0
        front_distance = 0.75 # 1.0

        # CASO 1: No hay nada con lo que chocar - BUSCAR PARED
        if self.regions['front']>front_distance and self.regions['front_left']>left_distance and self.regions['front_right']>right_distance:
            self.change_state(0)
        # CASO 2: Hay algo al frente - GIRAR A LA IZQUIERDA
        elif self.regions['front']<front_distance and self.regions['front_left']>left_distance and self.regions['front_right']>right_distance:
            self.change_state(1)
        # CASO 3: Hay algo al frente_derecha - SIGUE LA PARED
        elif self.regions['front']>front_distance and self.regions['front_left']>left_distance and self.regions['front_right']<right_distance:
            self.change_state(2)
        # CASO 4: Hay algo en el front_left - BUSCAR PARED
        elif self.regions['front']>front_distance and self.regions['front_left']<left_distance and self.regions['front_right']>right_distance:
            self.change_state(0)
        # CASE 5: There is something at the front and at the right - TURN LEFT
        elif self.regions['front']<front_distance and self.regions['front_left']>left_distance and self.regions['front_right']<right_distance:
            self.change_state(1)
        # CASO 5: Hay algo al frente ya la derecha - GIRAR A LA IZQUIERDA
        elif self.regions['front']<front_distance and self.regions['front_left']<left_distance and self.regions['front_right']>right_distance:
            self.change_state(1)
        # CASO 7: Hay algo al frente, al frente_izquierdo y al frente_derecho - GIRAR A LA IZQUIERDA
        elif self.regions['front']<front_distance and self.regions['front_left']<left_distance and self.regions['front_right']<right_distance:
            self.change_state(1)
        # CASO 8: Hay algo en el frente_izquierdo y en el frente_derecho - BUSCAR PARED
        elif self.regions['front']>front_distance and self.self.regions['front_left']<left_distance and self.regions['front_right']<right_distance:
            self.change_state(0)
        # Casp desconocido
        else: 
            rospy.loginfo(self.regions)

    def seguir_pared(self):

        self.state1 = 0
        self.state_dict_ = {
            0: 'buscar pared', # No existe ninguna pared en el rango establecido
            1: 'giro izquierda',  # Hay una pared, pero estamos demasiado cerca de la parte derecha, o giramos para tener la pared a la derecha
            2: 'seguimos la pared', # La pared esta a la derecha y no esta demasiado cerca
        }

        """
        Recibe la informacion que hace que el robot se mueva
        """

        print("Se esta siguiendo la pared")
        #rate=rospy.Rate(5)
        #move = Twist()
        while not rospy.is_shutdown():
             
            name=self.reconocimiento_de_voz(self.r,self.m,timeout=0.1,pantalla=False)
            self.lets_go()

            if name == "parar":   
                print("Paranis de seguir la pared")
                self.move_msg.linear.x = 0
                self.move_msg.angular.z = 0
                self.pub_vel.publish(self.move_msg)
                break

            else:
                print("El estado es",self.state1)
                if self.state1 == 0:
                    self.move_msg.linear.x = 0.2
                    self.move_msg.angular.z = -0.3 # Giro derecha

                elif self.state1 == 1:
                    self.move_msg.angular.z = 0.3 # Giro izquierda

                elif self.state1 == 2:
                    self.move_msg.linear.x = 0.2

                else:
                    rospy.logerr('Unknown state!')
            
            self.pub_vel.publish(self.move_msg)
            self.rate.sleep()
            
        # Callback definition

    @staticmethod 
    def active_cb():
        rospy.loginfo("Se esta procesando el objetivo")

    @staticmethod 
    def feedback_cb(feedback):
        #rospy.loginfo("Ubicacion actual: " + str(feedback))
        pass

    
    def done_cb(self,status, result):
        self.ejecutando_accion_ros=0
        if status == 3:
            rospy.loginfo("Objetivo alcanzado")
            self.ruta_posicion_alcanzada=1
        if status == 2 or status == 8:
            rospy.loginfo("Objetivo cancelado")
        if status == 4:
            rospy.loginfo("Objetivo abortado")
            
    def got_to(self,goal):

        
        self.ejecutando_accion_ros=1

        self.msg_goal.target_pose.header.frame_id = "map"
        self.msg_goal.target_pose.header.stamp = rospy.Time.now()

        self.msg_goal.target_pose.pose.position.x = goal.px
        self.msg_goal.target_pose.pose.position.y = goal.py
        self.msg_goal.target_pose.pose.position.z = goal.pz
        self.msg_goal.target_pose.pose.orientation.x = goal.ox
        self.msg_goal.target_pose.pose.orientation.y = goal.oy
        self.msg_goal.target_pose.pose.orientation.z = goal.oz
        self.msg_goal.target_pose.pose.orientation.w = goal.ow
        
        self.navclient.send_goal(self.msg_goal,self.done_cb,self.active_cb,self.feedback_cb)

        #finished =self.navclient.wait_for_result()

        # if not finished:
        #    rospy.logerr("Action server not available!")
        #else:
        #rospy.loginfo(self.navclient.get_result())

    @staticmethod
    def reconocimiento_de_voz(r,m,texto="Say something!",timeout=1.5,pantalla=True):

        with sr.Microphone() as source:
            
            r.adjust_for_ambient_noise(source)
            if pantalla:
                print(texto)
            #audio = r.listen(source,timeout=1.5, phrase_time_limit=1.5)
            try:
                audio = r.listen(source,timeout=timeout);
                text = r.recognize_google(audio,language="es-ES",show_all=True);
                
                if type(text)==type({}):
                    text=text['alternative'][0]['transcript']
                if  type(text)==type(''):
                    print('You said: {}'.format(text))
                    return text 
            except:
                if pantalla:
                    print('Sorry could not hear')

    def guardar_posicion(self,resetear=False):
        name=None
        while name==None:
            if not resetear:
                name=self.reconocimiento_de_voz(self.r,self.m,"Ponle nombre a la ubicacion")

            if (not (name in self.ubicaciones) and name!=None) or resetear:
                if resetear:
                    name=""
                odom_msg = rospy.wait_for_message('/odom', Odometry)
                dic_inicio={}
                dic_inicio["ubicacion"]=name
                dic_inicio["pos_x"]=odom_msg.pose.pose.position.x
                dic_inicio["pos_y"]=odom_msg.pose.pose.position.y
                dic_inicio["pos_z"]=odom_msg.pose.pose.position.z

                dic_inicio["orientacion_x"]=odom_msg.pose.pose.orientation.x
                dic_inicio["orientacion_y"]=odom_msg.pose.pose.orientation.y
                dic_inicio["orientacion_z"]=odom_msg.pose.pose.orientation.z
                dic_inicio["orientacion_w"]=odom_msg.pose.pose.orientation.w
                if not resetear:
                    self.ubicaciones[dic_inicio["ubicacion"]]=ubicacion_data(dic_inicio)
                else:
                    return ubicacion_data(dic_inicio)

    def control_mano(self):

        tiempos=deque()
        ruta_creada_v_a=deque()
        ruta_creada_v_a_str=deque()

        mp_drawing = mp.solutions.drawing_utils
        mp_hands = mp.solutions.hands

        cTime = time.time()
        pTime = cTime

        minVol = 0
        maxVol = 10

        # For webcam input:
        cap = cv2.VideoCapture(0)

        
        print("ESTAMOS PROBANDO EL CONTROL DE MANO")
        with mp_hands.Hands(
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5) as hands:

            while cap.isOpened() and not rospy.is_shutdown():
                vel_lineal=0
                vel_angular=0

                #name=self.reconocimiento_de_voz(self.r,self.m,"Ponle nombre a la ubicacion")
                if self.min_dist<self.dist_seguridad:
                    print("---------------------------------")
                    print("Paranis de seguir la pared")
                    print(self.min_dist,self.dist_seguridad)
                    print("---------------------------------")
                    self.move_msg.linear.x = 0
                    self.move_msg.angular.z = 0
                    self.pub_vel.publish(self.move_msg)
                    break

                else:
                    success, image = cap.read()
                    if not success:
                        print("Ignoring empty camera frame.")
                        # If loading a video, use 'break' instead of 'continue'.
                        continue


                    
                    
                    # Flip the image horizontally for a later selfie-view display, and convert
                    # the BGR image to RGB.
                    image = cv2.cvtColor(cv2.flip(image, 1), cv2.COLOR_BGR2RGB)
                    h,w,_=image.shape
                    #print(image.shape)
                    # To improve performance, optionally mark the image as not writeable to
                    # pass by reference.
                    image.flags.writeable = False
                    results = hands.process(image)

                    # Draw the hand annotations on the image.
                    image.flags.writeable = True
                    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
                    
                    if results.multi_hand_landmarks:
                        
                        for hand_landmarks in results.multi_hand_landmarks:
                            # print(lmList[4], lmList[8])
                            # x1, y1 = hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_TIP].x,hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_TIP].y

                            x1, y1 = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_MCP].x,hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_MCP].y 

                            #print(x1,y1)
                            x1, y1=int(x1*w), int(y1*h)
                            x2, y2 = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP].x, hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP].y 
                            #print(x2,y2)
                            x2, y2=int(x2*w), int(y2*h)
                            cx, cy = (x1+x2)//2, (y1+y2)//2
                            #print(cx, cy )
                            if cx!=0 and  cy !=0:
                                cv2.circle(image, (x1, y1), 10, (255, 0, 255), cv2.FILLED)
                                cv2.circle(image, (x2, y2), 10, (255, 0, 255), cv2.FILLED)
                                cv2.line(image, (x1, y1), (x2, y2), (255, 0, 255), 2)
                                cv2.line(image, (x1, 0), (x1, h), (0, 0, 255), 2)
                                cv2.line(image, (0, y1), (w, y1), (0, 0, 255), 2)
                                #cv2.circle(image, (cx, cy), 8, (255, 0, 255), cv2.FILLED)

                                length = math.hypot(x2-x1, y2-y1)
                                # print(length)


                                # Hand Range 25 - 250
                                # Vol Range -65 - 0
                                vol = np.interp(length, [25, 250], [minVol, maxVol])
                                volBar = np.interp(length, [25, 250], [350, 150])
                                volPer = np.interp(length, [25, 250], [0, 100])

                                print(int(volPer))
                                cv2.rectangle(image, (50, 150), (75, 350), (0,255,0), 2)
                                cv2.rectangle(image, (50, int(volBar)), (75, 350), (0, 255, 0), cv2.FILLED)
                                cv2.putText(image, f'{int(volPer)} %', (44, 390), cv2.FONT_HERSHEY_COMPLEX, 0.6, (200, 0, 0), 2)

                                
                                vel_lineal=volPer
                                vel_angular=np.arctan2(y2-y1,x2-x1)+np.pi/2
                                if vel_angular>np.pi:
                                    vel_angular-=2*np.pi
                                    if vel_angular<np.pi:
                                        vel_angular+=np.pi/2
                                        vel_angular=np.pi/2+vel_angular
                                        vel_lineal=-volPer
                                if vel_angular>np.pi/2:
                                    vel_angular-=np.pi/2
                                    vel_angular=np.pi/2-vel_angular
                                    vel_lineal=-volPer
                                
                                #if vel_angular>np.pi:
                                #    vel_angular-=2*np.pi

                                print("velocidad_lineal",(vel_lineal/100)*1)
                                print("velocidad_angualr",vel_angular*180/np.pi)

                                cv2.putText(image, f'{vel_angular*180/np.pi}', (x2+10, y2+10), cv2.FONT_HERSHEY_COMPLEX, 0.6, (200, 0, 0), 2)

                            mp_drawing.draw_landmarks(
                                image, hand_landmarks, mp_hands.HAND_CONNECTIONS)
                        
                    
                    cTime = time.time()
                    fps = 1 / (cTime - pTime)
                    tiempos.append(cTime - pTime)
                    pTime = cTime
                    
                    cv2.putText(image, f'FPS : {int(fps)}', (35, 50), cv2.FONT_HERSHEY_COMPLEX, 0.7, (255, 0, 0), 2)

                    cv2.imshow('hand_control', image)
                    if cv2.waitKey(5) & 0xFF == 27:
                        cv2.destroyWindow('hand_control')
                        break
                
                self.move_msg.linear.x = (vel_lineal/100)*0.5
                self.move_msg.angular.z = vel_angular
                if not(vel_lineal==0 and vel_angular==0):
                    ruta_creada_v_a.append([(vel_lineal/100)*0.5,vel_angular])
                    ruta_creada_v_a_str.append("_".join([str((vel_lineal/100)*0.5), str(vel_angular)]))
                self.pub_vel.publish(self.move_msg)   
                
                #self.rate.sleep()
            cap.release()
            cv2.destroyAllWindows()

        return ruta_creada_v_a,"p".join(ruta_creada_v_a_str),tiempos

    def crear_ruta(self):

        reset_positicion=self.guardar_posicion(True)
        ruta,ruta_str,tiempo=self.control_mano()
        self.got_to(reset_positicion)
        name=None
        while name==None:

            name=self.reconocimiento_de_voz(self.r,self.m,"Ponle nombre a la ruta")
            if (not (name in self.memoria_rutas) and name!=None and name!="parar" and name !="no guardar" ):
                self.memoria_rutas[name]={"inicio":reset_positicion,"ruta":ruta,"ruta_str":ruta_str,"espera":tiempo}
            if name == "parar" or name =="no guardar":
                break

    def ejecutar_ruta(self,ruta):
        
        cont=0
        #numordenes=len(ruta["ruta"])
        odenes_ruta=ruta["ruta"]
        numordenes=len(odenes_ruta)
        #print("Las ordenes",odenes_ruta)
        #print(cont,numordenes)
        print("Ejecutamos la ruta selecionada")
        tiempos_espera=ruta["espera"]
        while cont<numordenes:
                vel_lineal=odenes_ruta[cont][0]
                vel_angular=odenes_ruta[cont][1]
                #name=self.reconocimiento_de_voz(self.r,self.m,"Ponle nombre a la ubicacion")
                if self.min_dist<self.dist_seguridad:   
                    print("Paranis de seguir la pared")
                    self.move_msg.linear.x = 0
                    self.move_msg.angular.z = 0
                    self.pub_vel.publish(self.move_msg)
                    break
                
                
                self.move_msg.linear.x = vel_lineal
                self.move_msg.angular.z = vel_angular
                self.pub_vel.publish(self.move_msg)

                if cont<len(tiempos_espera):
                    #print("esperamos el tiempo necesario",tiempos_espera[cont])
                    time.sleep(tiempos_espera[cont])
                else:
                    time.sleep(np.mean(np.array(tiempos_espera)))
                cont+=1

        self.move_msg.linear.x = 0
        self.move_msg.angular.z = 0
        self.pub_vel.publish(self.move_msg)

    def que_ruta_ejecutar(self):
        old=None
        while not rospy.is_shutdown():
            print("las ubicaciones disponibles",self.memoria_rutas.keys())
            #print(self.memoria_rutas)
            
            if self.ruta_posicion_alcanzada and old!=None:
                self.ejecutar_ruta(self.memoria_rutas[old])
                self.ruta_posicion_alcanzada=0
            else:
                if not self.ejecutando_accion_ros:
                    orden=self.reconocimiento_de_voz(self.r,self.m,"Que ruta ejecutamos?")
                else:
                    orden=self.reconocimiento_de_voz(self.r,self.m,pantalla=False)
                if orden=="salir":
                    break
                
                else:
                    if orden!=None:
                        if orden in self.memoria_rutas:
                            old=orden
                            self.got_to(self.memoria_rutas[orden]["inicio"])
                            self.ejecutando_accion_ros=1
                            #self.ejecutar_ruta(self.memoria_rutas[orden])

    def up_load_ubicacion(self):
        diccionario_escribit={}
        
        with open(self.memoria_path, 'w', newline='') as csvfile:
            fieldnames = self.keys_save_ubicacion
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()
            
            for keys in self.ubicaciones.keys():
                estar=True
                while estar:
                    orden=self.reconocimiento_de_voz(self.r,self.m,texto="Quieres guardar la ubicacion "+str(keys))
                    if orden in {"SI","si","no","No"}:

                        break

                if orden in {"SI","si"}:
                    dic_inicio={}
                    val_data=self.ubicaciones[keys]
                    dic_inicio["ubicacion"]=keys
                    
                    dic_inicio["pos_x"]=val_data.px
                    dic_inicio["pos_y"]=val_data.py
                    dic_inicio["pos_z"]=val_data.pz

                    dic_inicio["orientacion_x"]=val_data.ox
                    dic_inicio["orientacion_y"]=val_data.oy
                    dic_inicio["orientacion_z"]=val_data.oz
                    dic_inicio["orientacion_w"]=val_data.ow
                    
                    writer.writerow(dic_inicio)

        with open(self.memoria_path_ruta, 'w', newline='') as csvfile:
            fieldnames = self.keys_save_ruta
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()
            
            for keys in self.memoria_rutas.keys(): 

                estar=True
                while estar:
                    orden=self.reconocimiento_de_voz(self.r,self.m,texto="Quieres guardar la ruta "+str(keys))
                    if orden in {"SI","si","no","No"}:

                        break

                if orden in {"SI","si"}:
                    dic_inicio={}
                    val_data=self.memoria_rutas[keys]["inicio"]
                    dic_inicio["ubicacion"]=keys
                    
                    dic_inicio["pos_x"]=val_data.px
                    dic_inicio["pos_y"]=val_data.py
                    dic_inicio["pos_z"]=val_data.pz

                    dic_inicio["orientacion_x"]=val_data.ox
                    dic_inicio["orientacion_y"]=val_data.oy
                    dic_inicio["orientacion_z"]=val_data.oz
                    dic_inicio["orientacion_w"]=val_data.ow
                    dic_inicio["ruta"]=self.memoria_rutas[keys]["ruta_str"]
                    dic_inicio["espera"]="_".join(list(map(str,self.memoria_rutas[keys]["espera"])))
                    writer.writerow(dic_inicio)

    def traer(self):

        reset_positicion=self.guardar_posicion(True)
        self.ruta_posicion_alcanzada=0
        self.got_to(self.ubicaciones["inicio"])
        print("ACCIONES parar,salir,continuar")

        while self.ejecutando_accion_ros:
            orden=self.reconocimiento_de_voz(self.r,self.m,pantalla=False)
            if orden == "parar" and self.ejecutando_accion_ros:   
                self.navclient.cancel_all_goals()
                self.ejecutando_accion_ros=0
            if orden == "salir" and self.ejecutando_accion_ros:  
                self.navclient.cancel_all_goals()
                self.ejecutando_accion_ros=0  
                break
            if orden=="continuar" and not self.ejecutando_accion_ros:
                self.got_to(self.ubicaciones["inicio"])
        if self.ruta_posicion_alcanzada:
            print("RECOGIENDO EL OBJETO")
            time.sleep(1.5)
            print("HEMOS RECOGIDO EL OBJETO")
        self.got_to(reset_positicion)
        while self.ejecutando_accion_ros:
            orden=self.reconocimiento_de_voz(self.r,self.m,pantalla=False)
            if orden == "parar" and self.ejecutando_accion_ros:   
                self.navclient.cancel_all_goals()
                self.ejecutando_accion_ros=0
            if orden == "salir" and self.ejecutando_accion_ros:  
                self.navclient.cancel_all_goals()
                self.ejecutando_accion_ros=0  
                break
            if orden=="continuar" and not self.ejecutando_accion_ros:
                self.got_to(self.ubicaciones["inicio"])
        
    def prueba(self):
        #in_ruta=0
        while not rospy.is_shutdown():
            print("-------------------------------------------------------------------------------------------------------------------")
            print("Aciones disponibles \n parar: para la ejecucion actual  \n guardar: la posicion actual  \n seguir:  sigue la pared por la derecha \n crear: crea una nueva nuta")
            print(" ruta: ejecuta una ruta almacenada \n acturalizar: permite volcar los datos recogidos en la ejecucion rutas y ubicaiones\n traer: mueve el robot hasta un lugar donde se supone que estara un dispositivo que lo carga con lo que deseamos")
            print(" SALIR : Termina la ejecucion de control")
            if not self.ejecutando_accion_ros:
                print("las ubicaciones disponibles",list(self.ubicaciones.keys()))
            #print(self.ubicaciones)
            #print("Resultado",self.navclient.get_result())
            print("-------------------------------------------------------------------------------------------------------------------")
            orden=self.reconocimiento_de_voz(self.r,self.m)

            if orden=="salir":
                print("Se detuvo El control del robot")
                self.navclient.cancel_all_goals()
                self.move_msg.linear.x = 0
                self.move_msg.angular.z = 0
                self.pub_vel.publish(self.move_msg)
                break
            else:
                if orden!=None:

                    if orden in self.ubicaciones:

                        if self.ejecutando_accion_ros:
                            self.navclient.cancel_all_goals()
                        self.got_to(self.ubicaciones[orden])
                        self.ejecutando_accion_ros=1

                    if orden == "parar" and self.ejecutando_accion_ros:   
                        print("Se detuvo la acción")
                        self.navclient.cancel_all_goals()
                        self.ejecutando_accion_ros=0
                        

                    if orden == "guardar posicion" or orden == "guardar":
                        print("Se  va aguardar el punto actual")
                        if self.ejecutando_accion_ros:
                            self.navclient.cancel_all_goals()
                            self.ejecutando_accion_ros=0
                        self.guardar_posicion()

                    if orden == "seguir pared" or orden == "seguir":
                        if self.ejecutando_accion_ros:
                            self.navclient.cancel_all_goals()
                            self.ejecutando_accion_ros=0
                        #self.seguir_pared()
                        pass

                    if orden == "crear":
                        print("Se  va a crear una nueva ruta ")
                        if self.ejecutando_accion_ros:
                            self.navclient.cancel_all_goals()
                            self.ejecutando_accion_ros=0

                        self.crear_ruta()
                        # Se termina si se mantiene esc por 5 sgundos

                    if orden == "ejecutar ruta" or orden == "ruta":
                        print("Se  va a ejecutar una ruta ya almacenada")
                        if self.ejecutando_accion_ros:
                            self.navclient.cancel_all_goals()
                            self.ejecutando_accion_ros=0
                            
                        self.que_ruta_ejecutar()

                    if orden== "actualizar":
                        print("Se  va a actualizar los datos de la memoria ROM")
                        self.up_load_ubicacion()

                    if orden== "traer":
                        print("Se  va Traer eL Objeto deseado")
                        self.traer()

        print("-------------------------------------------------------------------------------------------------------------------")

if __name__=="__main__":

    memoria_path=os.path.realpath(__file__).split("/")
    memoria_path[-1]='memoria/ubicaiones.csv'
    memoria_path="/".join(memoria_path)

    memoria_path_r=os.path.realpath(__file__).split("/")
    memoria_path_r[-1]='memoria/ruta.csv'
    memoria_path_r="/".join(memoria_path_r)

    a=agente(memoria_path=memoria_path,path=memoria_path_r)
    #a.got_to()
    a.prueba()