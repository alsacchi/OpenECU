import threading, time
switch = True
UUID = "0000ffe1-0000-1000-8000-00805f9b34fb"
class Telemetry(threading.Thread):
   
    def __init__(self, s):
        global switch
        switch = True
        self._s = s
        threading.Thread.__init__(self)

    async def run(self):
        global switch
        while switch:
            await self._s.write_gatt_char(UUID, b'i')
            info = await self._s.read_gatt_char(UUID)
            print(info)
            time.sleep(0.1)
    def stop(self):
        global switch
        switch = False
