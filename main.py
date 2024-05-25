import serial

arduino = serial.Serial("COM3", 9600)
while True:
    if arduino.inWaiting() > 0:
        data = arduino.readline().decode('utf-8')
        print(data, end='')
