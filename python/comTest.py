import serial
import time
import struct

def pack(value):
    return struct.pack('>B', value)

def main():
    ser = serial.Serial("/dev/ttyUSB0", 9600)
    time.sleep(3)
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    time.sleep(5)
    val = 100

    while True:
        val = val + 10
        ser.write(pack(1))
        ser.write(pack(100))
        ser.write(pack(0))
        ser.write(pack(val))
        time.sleep(1)
        if val == 150:
            val = 0

if __name__ == "__main__":
    main()
