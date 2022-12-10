import serial
 
ser = serial.Serial()
ser.baudrate = 115200
ser.port = '/dev/ttyUSB0'
ser.open()
 
values = bytearray([0x2a, 0x30, 0, 25, 25, 0x92, 0xa0, 0, 25, 25] + [0,0,0,0,0] * 8)
ser.write(values)
 
total = 0
 
while total < len(values):
    # print(ord(ser.read(1)))
    total=total+1
 
ser.close()