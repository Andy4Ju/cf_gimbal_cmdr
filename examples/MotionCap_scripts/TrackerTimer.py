from time import time

class TrackTimer:
    def __init__(self, vehicle):
        self.fctimecorrected = None
        self.fcoffset = None
        self.sendTimeStamp = None
        self.flightComputerTime = None
        self.debug = False
        self.vehicle = vehicle
        @self.vehicle.on_message("SYSTEM_TIME")
        def listener_systemtime(subself, name, message):
            self.update()

        @self.vehicle.on_message("TIMESYNC")
        def listener_timesync(subself, name, message):
            if message.ts1 == self.sendTimeStamp:
                self.send_timesync(int(time() * 1e+9), message.tc1)
                # flightTime = (int(time() * 1e+9) - message.ts1)
                # print flightTime
            # else:
            #     print message.ts1, int(time() * 1e+9)

    def send_timesync(self, tc, ts):
        msg = self.vehicle.message_factory.timesync_encode(
            tc,          # tc1
            ts          # ts1
        )
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()
    def update(self):
        self.sendTimeStamp = int(time() * 1e+9)
        self.send_timesync(0, self.sendTimeStamp)

    def actual(self):
        return self.fctimecorrected
    def estimate(self):
        if self.fctimecorrected:
            newdiff = int(time() * 1e+9) - self.computerTimeStamp
            return self.fctimecorrected + newdiff
        else:
            return None