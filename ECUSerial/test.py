import asyncio
from bleak import BleakClient
from bleak import _logger as logger
from struct import unpack, pack

MAC_ADDR = "D4:36:39:5E:40:ED"
UUID = "0000ffe1-0000-1000-8000-00805f9b34fb"
map = bytearray()
counter = 0
def read_map(sender: int, data: bytearray):
    global map, counter
    print(f"NOTIFICATION! {counter} {len(data)}")
    counter += 1
    map += data


async def run():
    global map
    async with BleakClient(MAC_ADDR) as ser:
        x = await ser.is_connected()
        logger.info("Connected: {0}".format(x))
        await ser.start_notify(UUID, read_map)
        data = " "
        while(data != ""):
            data = input(">")
            map = bytearray()
            print("Trying to read injectorMAP")
            await ser.write_gatt_char(UUID, data.encode())
            await asyncio.sleep(5.0)
            print(map)
            print(len(map))
        print("Closing Serial comunication!")

        await ser.disconnect()

loop = asyncio.get_event_loop()
# loop.set_debug(True)
loop.run_until_complete(run())