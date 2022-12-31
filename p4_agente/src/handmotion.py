#!/usr/bin/env python3

# NOTE: this example requires PyAudio because it uses the Microphone class
#https://stackoverflow.com/questions/14257598/what-are-language-codes-in-chromes-implementation-of-the-html5-speech-recogniti
#https://stackoverflow.com/questions/7088672/pyaudio-working-but-spits-out-error-messages-each-time
# Para reconocer la voz entrada
import speech_recognition as sr
# sonido del asistende virtual
# pip install gTTS
from gtts import gTTS

import os 
from std_msgs.msg import String
import rospy

import re
from unicodedata import normalize
from gtts import gTTS

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



def reconocimiento_de_voz(r,m):
  with sr.Microphone() as source:
    
    r.adjust_for_ambient_noise(source)
    print("Say something!")
    #audio = r.listen(source,timeout=1.5, phrase_time_limit=1.5)
    try:
      audio = r.listen(source,timeout=1.5)
      text = r.recognize_google(audio,language="es-ES")
      print('You said: {}'.format(text))
      return(text)
    except:
      print('Sorry could not hear')
  
# obtain audio from the microphone
r = sr.Recognizer()
m=sr.Microphone() 
#print(sr.Microphone(device_index=1))
pub = rospy.Publisher('orden', String, queue_size=10)
rospy.init_node('talker', anonymous=True)
rate = rospy.Rate(10) # 10

while not rospy.is_shutdown():
    orden=reconocimiento_de_voz(r,m)
    if orden=="salir":
      break
    else:
      #print('la orden es  {}'.format(orden))
      orden=re.sub(r"([^n\u0300-\u036f]|n(?!\u0303(?![\u0300-\u036f])))[\u0300-\u036f]+", r"\1",normalize( "NFD", orden), 0, re.I)
      #print('la orden es  {}'.format(orden))
      pub.publish(orden)