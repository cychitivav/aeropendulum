from machine import Pin
import time
import numpy as np

p2 = Pin(2, Pin.OUT)
count = 0

while True:
    count+= 1
    p2.on()
    time.sleep(1)
    p2.off()
    time.sleep(1)

    print(count)