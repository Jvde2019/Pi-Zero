#!/usr/bin/python3
# -*- coding: utf-8 -*-

#OLED import
from oled_text import OledText, Layout32
from board import SCL, SDA
import busio
import time

#DHT22 import
import Adafruit_DHT
from time import sleep

#shutdown imports
import RPi.GPIO as GPIO
import os

#shutdown
#buttonPin = 21

#GPIO.setmode(GPIO.BCM)
#GPIO.setup(buttonPin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

#last_state = True
#input_state = True

#shutdown interrupt
Counter = 0
GPIO.setup(21, GPIO.IN, pull_up_down = GPIO.PUD_UP)

def Interrupt(Channel):
    global Counter
    Counter = Counter + 1
    print ("Counter " + str(Counter))
    print("Shutdown")
    oled.text("Shutdown",1)
    oled.text("",2)
    os.system('sudo shutdown -h now')
    time.sleep(0.05)
    GPIO.cleanup()

GPIO.add_event_detect(21, GPIO.RISING, callback = Interrupt, bouncetime = 200)

i2c = busio.I2C(SCL, SDA)

""" Examples for a 128x32 px SSD1306 oled display. For more details see the 64px examples """

# Instantiate the display, passing its dimensions (128x64 or 128x32)
oled = OledText(i2c, 128, 32)

sensor = Adafruit_DHT.DHT22
# DHT22 sensor connected to GPIO12.
pin = 12
print("[press ctrl+c to end the script]")  
try: # Main program loop
    while True:
        
        #input_state = GPIO.input(buttonPin)
        #if (not input_state):
        #    print("Shutdown")
        #    os.system('sudo shutdown -h now')
        #    time.sleep(0.05)
        #    GPIO.cleanup()
        humidity, temperature = Adafruit_DHT.read_retry(sensor,pin)
        sleep(2.5)
        if humidity is not None and temperature is not None:
            print("Temp={0:0.1f}*C  Humidity={1:0.1f}%".format(temperature, humidity))
            oled.layout = Layout32.layout_2medium()
            oled.auto_show = False
            text1 = "Temperatur {0:0.1f} °C".format(temperature)
            oled.text(text1,1)
            text2 = "Feuchte {0:0.1f} %r.F.".format(humidity)
            oled.text(text2,2)
            oled.show()
            oled.auto_show = True
            time.sleep(2) 
        else:
            print("Failed to get reading. Try again!")
# Scavenging work after the end of the program
except KeyboardInterrupt:
    print("Script end!")