from dronekit import connect
import time
vehicle = connect('127.0.0.1:14550', wait_ready=True, baud=57600)

pitch= 0.0
yaw= 0.0
roll= 0.0
try:
    while True:
        attitude = vehicle.attitude
        
        pitch = attitude.pitch
        yaw = attitude.yaw
        roll = attitude.roll
        
        print("Pitch: ", pitch)
        print("yaw: ", yaw)
        print("Roll: ", roll)
     
        
        print("Mode: ", vehicle.mode)
        
        
        print("Vehicle Version: ", vehicle.version)
        print("---------------------")
        

        time.sleep(5)

except KeyboardInterrupt:   # ctrl+c
    print("Exiting")

finally:
    vehicle.close()
