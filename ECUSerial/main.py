import asyncio
import sys
import numpy as np
from telemetry import Telemetry
from bleak import BleakClient
from bleak import _logger as logger
from struct import unpack, pack

MAC_ADDR = "D4:36:39:5E:40:ED"
UUID = "0000ffe1-0000-1000-8000-00805f9b34fb"


def read_map(sender: int, data: bytearray):
    print("NOTIFICATION!")
    print(data)


async def run():
    async with BleakClient(MAC_ADDR) as ser:
        x = await ser.is_connected()
        logger.info("Connected: {0}".format(x))
        await ser.start_notify(UUID, read_map)
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
                await ser.write_gatt_char(UUID, b'w')
                await ser.write_gatt_char(UUID, pack("<h", offset) + pack("<h", len(buffer)))
                await ser.write_gatt_char(UUID, buffer)
                #print(ser.write(b'w' + pack("<h", offset) + pack("<h", len(buffer)) + buffer))
                print("Wrote to the EEPROM")
            elif(data == 'r'):
                print("Trying to read from EEPROM")
                await ser.write_gatt_char(UUID, b'r')
                eeprom_buf = await ser.read_gatt_char(UUID)
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
                await ser.write_gatt_char(UUID, b'm')
                
                
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
                
            #else:
                #ser.write(str.encode(data))

        print("Closing Serial comunication!")

        await ser.disconnect()

loop = asyncio.get_event_loop()
# loop.set_debug(True)
loop.run_until_complete(run())