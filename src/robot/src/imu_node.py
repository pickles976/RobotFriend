import serial
import json

ser = serial.Serial('/dev/ttyACM0')  # open serial port
ser.baudrate = 9600
print(ser.name)         # check which port was really used

while True:
    imu_string = ser.readline().decode('utf8')
    print(imu_string)
    try: 
        imu_data = json.loads(imu_string)
        print(imu_string)
    except:
        print("String not JSON format: %s"%imu_string)

ser.close()             # close port