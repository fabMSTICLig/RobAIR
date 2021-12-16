#!/usr/bin/python3

from gpiozero import Button
import subprocess 
from time import sleep
from rpi_ws281x import *

LED_COUNT      = 1      # Number of LED pixels.
LED_PIN        = 18      # GPIO pin connected to the pixels (18 uses PWM!).
LED_FREQ_HZ    = 800000  # LED signal frequency in hertz (usually 800khz)
LED_DMA        = 10      # DMA channel to use for generating signal (try 10)
LED_BRIGHTNESS = 15     # Set to 0 for darkest and 255 for brightest
LED_INVERT     = False   # True to invert the signal (when using NPN transistor level shift)
LED_CHANNEL    = 0       # set to '1' for GPIOs 13, 19, 41, 45 or 53

PIN_BUTTON     = 10

cmd_is_active = ["env", "XDG_RUNTIME_DIR=/run/user/1000", "sudo", "--preserve-env=XDG_RUNTIME_DIR", "-u", "ubuntu", "systemctl", "is-active", "--user", "follow_me_robair"]
cmd_start =     ["env", "XDG_RUNTIME_DIR=/run/user/1000", "sudo", "--preserve-env=XDG_RUNTIME_DIR", "-u", "ubuntu", "systemctl", "start", "--user", "follow_me_robair"]
cmd_stop =      ["env", "XDG_RUNTIME_DIR=/run/user/1000", "sudo", "--preserve-env=XDG_RUNTIME_DIR", "-u", "ubuntu", "systemctl", "stop", "--user", "follow_me_robair"]


BLUE = Color(0,8,192)
GREEN = Color(0,192,8)

# Init button
button = Button(PIN_BUTTON)

# Create NeoPixel object with appropriate configuration.
strip = Adafruit_NeoPixel(LED_COUNT, LED_PIN, LED_FREQ_HZ, LED_DMA, LED_INVERT, LED_BRIGHTNESS, LED_CHANNEL)

strip.begin()
status = subprocess.run(cmd_is_active, stdout=subprocess.PIPE).stdout.decode('utf-8')[:-1]

if status == "active":
    strip.setPixelColor(0, BLUE)
else:
    strip.setPixelColor(0, GREEN)
strip.show() 

while True:
    button.wait_for_press()
    print("button pressed")
    status = subprocess.run(cmd_is_active, stdout=subprocess.PIPE).stdout.decode('utf-8')[:-1]
    print("status: ", status)
    if status == "active":
        subprocess.run(cmd_stop)
        print("stopped now")
        strip.setPixelColor(0, GREEN)
        strip.show() 
    elif status == "inactive":
        subprocess.run(cmd_start)
        print("started now")
        strip.setPixelColor(0, BLUE)
        strip.show() 
    else:
        print("return error!")
    sleep(2)
strip.end()
