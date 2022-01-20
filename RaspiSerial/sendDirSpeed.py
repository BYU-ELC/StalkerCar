import serial
import time

if __name__ == '__main__':
    ser = serial.Serial("/dev/ttyUSB0", 19200, timeout=1);
    ser.reset_input_buffer()
    ser.write(b"300100")
    ser.close()