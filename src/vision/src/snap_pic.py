from picamera import PiCamera
import time
from time import sleep

WIDTH, HEIGHT = 1920, 1080

camera = PiCamera()
camera.resolution = (WIDTH, HEIGHT)
camera.rotation = 180
camera.start_preview()
sleep(3)
data= time.strftime("%Y-%b-%d_(%H%M%S)")
camera.capture('%s.jpg'%data)  
camera.stop_preview()