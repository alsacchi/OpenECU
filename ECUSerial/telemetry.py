import threading, time
switch = True
class Telemetry(threading.Thread):
   
    def __init__(self, s):
        global switch
        switch = True
        self._s = s
        threading.Thread.__init__(self)

    def run(self):
        global switch
        while switch:
            self._s.write(b'i')
            info = self._s.read(5*2 + 4)
            print(info)
            time.sleep(0.1)
    def stop(self):
        global switch
        switch = False
