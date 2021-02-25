from telemetry import Telemetry
import serial
import numpy as np
from struct import unpack, pack
ser = serial.Serial("COM4", 9600, timeout=5)
matrix_length = 43 * 10 * 2
#matrix = np.array([[0 for x in range(10)] for y in range(43)], dtype=np.dtype('<h'))
matrix = np.loadtxt('cMAP.txt', dtype=np.dtype('<h'), delimiter=',')
throttle_indexs = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
data = " "
toggle = False
while(data is not ""):
    data = input(">")
    if(data == 'w'):
        buffer = matrix.tobytes()
        offset = 0
        print(f"Writing datalen({len(buffer)}): to EEPROM")
        print(ser.write(b'w' + pack("<h", offset) + pack("<h", len(buffer)) + buffer))
        print("Wrote to the EEPROM")
    elif(data == 'r'):
        print("Trying to read from EEPROM")
        print(ser.write(b'r'))
        eeprom_buf = ser.read(matrix_length)
        counter = 0
        rpm = 0
        print("      Throttle", end="\nRPM")
        fmt = "{:<9}" * (len(throttle_indexs) + 1)
        print(fmt.format("", *throttle_indexs))
        for i in range(0,430*2,20):
            print("{:05} {}".format(rpm, [f'{unpack("<h", eeprom_buf[i:i+20][j:j+2])[0]:05}' for j in range(0, 20, 2)]))
            counter += 1
            rpm += 250
        print(counter)
    elif(data == 'm'):
        print("Trying to read injectorMAP")
        print(ser.write(b'm'))
        injector_map = ser.read(matrix_length)
        counter = 0
        rpm = 0
        print("      Throttle", end="\nRPM")
        fmt = "{:<9}" * (len(throttle_indexs) + 1)
        print(fmt.format("", *throttle_indexs))
        for i in range(0,430*2,20):
            print("{:05} {}".format(rpm, [f'{unpack("<h", injector_map[i:i+20][j:j+2])[0]:05}' for j in range(0, 20, 2)]))
            counter += 1
            rpm += 250
        print(counter)
    elif(data == 'c'):
        counter = 0
        rpm = 0
        print("      Throttle", end="\nRPM")
        fmt = "{:<9}" * (len(throttle_indexs) + 1)
        print(fmt.format("", *throttle_indexs))
        for i in range(0,430*2,20):
            print("{:05} {}".format(rpm, [f'{unpack("<h", matrix.tobytes()[i:i+20][j:j+2])[0]:05}' for j in range(0, 20, 2)]))
            counter += 1
            rpm += 250
        print(counter)
    elif(data == 'i'):
        if(not toggle):
            print("Reading realtime data")
            thread = Telemetry(ser)
            thread.start()
            toggle = not toggle
        else:
            print("Stopping telemetry")
            thread.stop()
            toggle = not toggle
        
    else:
        ser.write(str.encode(data))

print("Closing Serial comunication!")

ser.close()