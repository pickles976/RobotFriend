from picamera import PiCamera
import time
from time import sleep

camera = PiCamera()
camera.resolution = (1920, 1080)
camera.rotation = 180
camera.start_preview()
sleep(3)
data= time.strftime("%Y-%b-%d_(%H%M%S)")
camera.capture('./robot_img/%s.jpg'%data)  
camera.stop_preview()