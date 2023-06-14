# Librerías RPi para raspberry
from COD1 import alti
import RPi.GPIO as GPIO                     ## Importar libreria GPIO.
import time                                 ## Importar libreria time para delay.

# Configuración para pines
GPIO.setmode(GPIO.BMC)                    ## Configuracion en modo Board la raspberry
#GPIO.setup(37, GPIO.IN)                    ## Pin 37 de la placa configurado como salida
GPIO.setup(22, GPIO.OUT)                    ## PiN 22 de la placa configurado como salida

pwm2=GPIO.PWM(22,50)                        ## 
pwm2.start(5)                               ## Posiciona el servomotor en la posicion 5

angle1=10
duty1= float(angle1)/10 + 2                 ## Conversion de angulo a Duty Cycle

angle2=160
duty2= float(angle2)/10 + 2.5               ## Conversion de angulo a Duty Cycle

##############################################################################################

# This is a sample Python script.
from dronekit import connect, VehicleMode
import serial
import time
#import matplotlib.pyplot as plt

##Connection with PIXHAWK
connection_string = "COM3"
baud_rate = 115000
#--- Now that we have started the SITL and we have the connection string (basically the ip and udp port)...

print(">>>> Connecting with the UAV <<<")
vehicle = connect(connection_string, baud=baud_rate, wait_ready=True)     #- wait_ready flag hold the program untill all the parameters are been read (=, not .)

#-- Read information from the autopilot:
#- Version and attributes
vehicle.wait_ready('autopilot_version')
print('Autopilot version: %s'%vehicle.version)
#- Does the firmware support the companion pc to set the attitude?
print('Supports set attitude from companion: %s'%vehicle.capabilities.set_attitude_target_local_ned)
#- Read the actual position
print('Position: %s'% vehicle.location.global_relative_frame)
#- Read the actual attitude roll, pitch, yaw
print('Attitude: %s'% vehicle.attitude)
#- Read the actual velocity (m/s)
print('Velocity: %s'%vehicle.velocity) #- North, east, down
#- When did we receive the last heartbeat
print('Last Heartbeat: %s'%vehicle.last_heartbeat)
#- Is the vehicle good to Arm?
print('Is the vehicle armable: %s'%vehicle.is_armable)
#- Which is the total ground speed?   Note: this is settable
print('Groundspeed: %s'% vehicle.groundspeed) #(%)
#- What is the actual flight mode?    Note: this is settable
print('Mode: %s'% vehicle.mode.name)
#- Is the vehicle armed               Note: this is settable
print('Armed: %s'%vehicle.armed)
#- Is thestate estimation filter ok?
print('EKF Ok: %s'%vehicle.ekf_ok)
#/BatteryStatus>
#Extrayendo las coordenadas GPS del PIXHAWK
print('Battery status: %s'%vehicle.battery)
print('Pos Lat: %s '%vehicle.location.global_relative_frame.lat)
print('Pos Long: %s '%vehicle.location.global_relative_frame.lon)
print('Pos Alt: %s '%vehicle.location.global_relative_frame.alt)
#----- Adding a listener
#-- dronekit updates the variables as soon as it receives an update from the UAV
#-- you can define a callback function for predefined messages or define one for
#-- any mavlink message


while True:
            print("Basic Pre-Arm checks")
            # "Don't Try to arm until autopilot is Ready"
            while not (vehicle.is_armable):
                print("Waiting for vehicle to initialise... ")
                time.sleep(.1)
                alti = vehicle.location.global_relative_frame.alt
                if alti>=5:
                    pwm2.ChangeDutyCycle(duty2)
                    time.sleep(0.8)
                else:
                    pwm2.ChangeDutyCycle(duty1)
                    time.sleep(0.8)

                time.sleep(1)
                GPIO.cleanup()


            print("Arming motors")
            vehicle.armed = True
            # Copter should arm in stabilze mode and then auto...
            vehicle.mode = VehicleMode("AUTO")
            # to give it a push, wi will override the throttle channel:
            # print("How much throttle do I have?: ", vehicle.channels['3'] )
            time.sleep(1)
            vehicle.channels.overrides['3'] = 1053

