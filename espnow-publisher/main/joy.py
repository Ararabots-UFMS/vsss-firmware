import pygame
import math
from time import sleep
# from encoder import Encoder
# from sender_bt import Sender
import socket

from pygame import key, K_LEFT, K_RIGHT, K_UP, K_DOWN, K_SPACE
from utils.json_handler import JsonHandler
from collections import OrderedDict
from threading import Thread
import serial
 
ser = serial.Serial()
ser.baudrate = 115200
ser.port = '/dev/ttyUSB0'
ser.open()

values = [0x2a, 0x30, 0, 0, 0, 0x92, 0xa0, 0, 0, 0] + [0,0,0,0,0] * 8
stop_values = bytearray([0x2a, 0x30, 3, 0, 0, 0x92, 0xa0, 0, 0, 0] + [0,0,0,0,0] * 8)
ser.write(bytearray(values))
print(values)
pygame.init()

MAX_SPEED = 70

done = False

def worker():
    while not done:
        print(values)
        ser.write(bytearray(values))
        sleep(1/60)

t = Thread(target=worker)
t.daemon = True
t.start()

def show_addresses(bluetooths) -> None:
    for i, (name, bluetooth) in enumerate(bluetooths.items()):
        print("%02d) %20s\t%s" % (i, name, bluetooth))

def solve_mac(argument: str, bluetooths) -> str:
    if ':' in argument or '-' in argument:
        return argument
    else:
        index = int(argument)
        mac = bluetooths[list(bluetooths.keys())[index]]
        return mac   

def send_packet(packet: str, sock: socket.socket) -> None:
    packet_bytes = bytes([int(b) for b in packet.split(' ')])
    sock.send(packet_bytes)

if __name__ == "__main__":

    window = pygame.display.set_mode((300, 300))
    window.fill(0)
    pygame.display.flip()    

    accelX = 0
    accelY = 0
    clock = pygame.time.Clock()

    while not done:
        try:
            pygame.event.get()
            keys = pygame.key.get_pressed()
            # Axis 1 = eixo Y
            # Axis 0 = eixo x      

            axisY = keys[K_RIGHT] * -1.0 + keys[K_LEFT] * 1.0
            axisX = keys[K_UP] * 1.0 + keys[K_DOWN] * -1.0

            if abs(axisX) > 0.01:
                if abs(accelX) > 500.0:
                    if accelX > 0:
                        accelX = 500.0
                    else:
                        accelX = -500.0
                else:    
                    accelX += axisX*25
            else:
                accelX = 0

            if abs(axisY) > 0.01:
                accelY += -axisY*50
                if abs(accelY) > 1000.0: 
                    if accelY > 0:
                        accelY = 1000.0
                    else:
                        accelY = -1000.0
            else:
                if abs(accelY) -200 < 0:
                    accelY = 0
                else:
                    if accelY > 0:
                        accelY-=200.0
                    else:
                        accelY+=200.0

            _accelX = accelX * (0.002) * MAX_SPEED
            _accelY = accelY * (0.001) * MAX_SPEED

            speedR = _accelY+_accelX
            speedL  = _accelY-_accelX

            # speedL = int(-valueL * MAX_SPEED)
            # speedR = int(-valueR * MAX_SPEED) 

            #print("ESQ, value: {}, speed: {}".format(valueL, speedL))
            #print("DIR, value: {}, speed: {}".format(valueR, speedR))
            
            # Codificação de comandos
            # Quatro casos, para cada combinação de posições            
            
            if speedL >= 0 and speedR >= 0:
                # message = Encoder.encode_motor_pkg(1,0,0,0,speedL,speedR)
                # print("td positivo")
                opcode = 0
            elif speedL <= 0 and speedR <= 0:
                #speedL *= -1
                #speedR *= -1
                # message = Encoder.encode_motor_pkg(0,1,0,0,speedL,speedR)
                opcode = 3
            elif speedL >= 0 and speedR <= 0:
                #speedR *= -1
                # message = Encoder.encode_motor_pkg(0,0,0,0,speedL,speedR)
                opcode = 2
            else:
                #speedL *= -1
                # message = Encoder.encode_motor_pkg(1,1,0,0,speedL,speedR)
                opcode = 1

            speedL = abs(int(speedL)) & 255
            speedR = abs(int(speedR)) & 255

            if keys[K_SPACE]:
                speedL = 255
                speedR = 255

            for i in range(10):
                values[i*5+2] = opcode
                values[i*5+3] = speedL
                values[i*5+4] = speedR
            
            clock.tick(20)
            # sleep(0.5)
            
            done = keys[pygame.K_q]
            pygame.display.flip()
            
        except Exception as e:
            print("Vish: ", e)
            ser.close()
            # sender.closeConnection()


            break

        ser.write(stop_values)
        # sock.close()