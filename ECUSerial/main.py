import serial, time
from struct import unpack, pack
ser = serial.Serial("COM5", 9600)
matrix_length = 860

buffer = b'\x43'*matrix_length
offset = 0
print(f"Writing datalen({len(buffer)}): to EEPROM")
print(ser.write(b'w' + pack("<h", offset) + pack("<h", len(buffer)) + buffer))

time.sleep(1)

print("Wrote to the EEPROM")

print("Trying to read from EEPROM")
print(ser.write(b'r'))
eeprom_buf = ser.read(matrix_length)
counter = 0
for i in range(0,430*2,20):
        print(eeprom_buf[i:i+20].hex())
        counter += 1

print(counter)

print("Closing Serial comunication!")

ser.close()