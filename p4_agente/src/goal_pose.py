#!/usr/bin/env python
import cv2
import mediapipe as mp
import time
import rospy
from std_msgs.msg import String
import math
import numpy as np

mp_drawing = mp.solutions.drawing_utils
mp_hands = mp.solutions.hands

cTime = time.time()
pTime = cTime

minVol = 0
maxVol = 10

def status(input):
    #pub.publish(input)
    pass
# For webcam input:
cap = cv2.VideoCapture(0)
pub = rospy.Publisher('status_hand', String, queue_size=10)
rospy.init_node('talker', anonymous=True)
rate = rospy.Rate(10) # 10
with mp_hands.Hands(
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5) as hands:

    while cap.isOpened():
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

                print(x1,y1)
                x1, y1=int(x1*w), int(y1*h)
                x2, y2 = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP].x, hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP].y 
                print(x2,y2)
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
        pTime = cTime

        cv2.putText(image, f'FPS : {int(fps)}', (35, 50), cv2.FONT_HERSHEY_COMPLEX, 0.7, (255, 0, 0), 2)

        cv2.imshow('hand control', image)
        if cv2.waitKey(5) & 0xFF == 27:
            break
cap.release()