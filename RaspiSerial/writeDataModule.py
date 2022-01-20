import serial
import time

def openSerialConnection():
    ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
    ser.reset_input_buffer()
    return ser

def writeData(speed, direction, serialPort):
    serialPort.write(b"300100\n")
        

def readData(serialPort):
    line = serialPort.readline().decode('utf-8').rstrip()
    time.sleep(1)
    return line

if __name__ == '__main__':
    ser = openSerialConnection()
    while True:
        writeData(100, 300, ser)
        print(readData(ser))