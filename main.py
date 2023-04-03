import machine
from machine import ADC, Pin


class Motor:
    def __init__(self, mid, dir1, dir2, pwm_pin):
        self.id = mid
        self.dir1 = Pin(dir1, Pin.OUT)
        self.dir2 = Pin(dir2, Pin.OUT)
        self.pos = 0
        self.target_pos = 0
        self.dir = MOTOR_OFF
        self.speed = 30000
        self.pwm = machine.PWM(Pin(pwm_pin))
        self.pwm.freq(500)

        self.state = 0

    def move_to(self, target):
        self.target_pos = target
        if (self.pos - self.target_pos) > 0:
            self.dir = MOTOR_CW
        else:
            self.dir = MOTOR_CCW
        self.state = 2

    def move(self, direction):
        self.dir = direction
        self.state = 1

    def stop(self):
        self.dir = MOTOR_OFF
        self.state = 0

    def update_pos(self, cnts):
        self.pos = cnts

    def update(self, cnts):
        self.pwm.duty_u16(self.speed)
        self.dir1.value(self.dir[0])
        self.dir2.value(self.dir[1])
        self.update_pos(cnts)
        if self.state == 2:
            if abs(self.pos - self.target_pos) < 10:
                self.stop()


resistorPin = Pin(26, Pin.IN, Pin.PULL_DOWN)
fsrADC = ADC(resistorPin)

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

MOTOR_CCW = [1, 0]
MOTOR_CW = [0, 1]
MOTOR_OFF = [0, 0]
motorMdir = MOTOR_OFF
motorSdir = MOTOR_OFF
POS_START = 0
motorM = Motor(0, 10, 11, 12)
motorS = Motor(1, 13, 14, 16)
FSR_THRESHOLD = (0.8 * 65535)
motorM.move_to(800)
motorS.move_to(800)
while True:
    print(cntM, cntS)
    force = fsrADC.read_u16()
    if force > FSR_THRESHOLD:
        motorM.stop()
        motorS.stop()
    motorM.update(cntM)
    motorS.update(cntS)

