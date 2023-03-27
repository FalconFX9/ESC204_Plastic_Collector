import machine
from machine import Pin


class Motor:
    def __init__(self, mid, dir1, dir2, pwm_pin, hA, hB):
        self.id = mid
        self.dir1 = dir1
        self.dir2 = dir2
        self.hA = hA
        self.hB = hB
        self.pos = 0
        self.target_pos = 0
        self.dir = MOTOR_OFF
        self.speed = 1000
        self.pwm = machine.PWM(pwm_pin)
        self.pwm.freq(500)
        self.pwm.duty_u16(self.speed)

    def move_to(self):
        if abs(self.pos - self.target_pos) > 10:
            if (self.pos - self.target_pos) > 0:
                self.dir = MOTOR_CW
            else:
                self.dir = MOTOR_CCW
        else:
            self.dir = MOTOR_OFF




hallMAid = 5
hallMBid = 6
hallSAid = 7
hallSBid = 8
cntM = 0
cntS = 0


hallMA = Pin(hallMAid, Pin.IN)
hallMB = Pin(hallMBid, Pin.IN)
hallSA = Pin(hallSAid, Pin.IN)
hallSB = Pin(hallSBid, Pin.IN)


def encode_M(pin: Pin):
    global cntM, hallMB
    if hallMB.value():
        cntM += 1
    else:
        cntM -= 1


def encode_S(pin: Pin):
    global cntS, hallSB
    if hallSB.value():
        cntS += 1
    else:
        cntS -= 1


hallMA.irq(trigger=Pin.IRQ_FALLING, handler=encode_M)
hallSA.irq(trigger=Pin.IRQ_FALLING, handler=encode_S)

MOTOR_CW = [1, 0]
MOTOR_CCW = [0, 1]
MOTOR_OFF = [0, 0]
motorMdir = MOTOR_OFF
motorSdir = MOTOR_OFF

while True:
    pass