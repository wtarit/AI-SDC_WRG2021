class Motor:
    def __init__(self, run, print_error, port = "/dev/ttyUSB0"):
        self.run = run
        self.port = port
        self.print_error = print_error
        self.last_error = 0
        self.MAX_SPEED = 127
        if run:
            import serial
            self.ser = serial.Serial(self.port, 115200)

    def motor(self, speed1, speed2):
        speed1 = speed1 + 127
        speed2 = speed2 + 127
        if(speed1 > 254):
            speed1 = 254
        elif(speed1 < 0):
            speed1 = 0
        if(speed2 > 254):
            speed2 = 254
        elif(speed2 < 0):
            speed2 = 0

        if self.run:
            self.ser.write(bytearray([int(speed1), int(speed2)]))

    def pid_control(self, errors, base_speed, KP, KD):
        derivative = errors - self.last_error
        output = (KP * errors) + (KD * derivative)
        # lmspeed = base_speed + errors
        # rmspeed = base_speed - errors
        lmspeed = base_speed + output
        rmspeed = base_speed - output
        if lmspeed > self.MAX_SPEED:
            lmspeed = self.MAX_SPEED
        elif lmspeed < 0:
            lmspeed = 0

        if rmspeed > self.MAX_SPEED:
            rmspeed = self.MAX_SPEED
        elif rmspeed < 0:
            rmspeed = 0

        if self.print_error:
            print(errors)
        self.motor(lmspeed, rmspeed)

if __name__ == "__main__":
    import time
    import sys
    motor = Motor(run = True, print_error=False)
    try:
        while True:
            motor.motor(70,70)
            time.sleep(1)

    except KeyboardInterrupt:
        motor.motor(0,0)
        sys.exit()