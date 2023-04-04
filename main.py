import machine
import time
from machine import ADC, Pin


class A4988Stepper:
    def __init__(self, mid, step_pin, dir_pin, en_pin, steps_per_rev):
        self.id = mid
        self.step_pin = Pin(step_pin, Pin.OUT)
        self.dir_pin = Pin(dir_pin, Pin.OUT)
        self.en_pin = Pin(en_pin, Pin.OUT)
        self.dir = 0
        self.steps_per_rev = steps_per_rev
        self.moving = False
        self.speed = 0
        self.time_between_steps = 0
        self.time_next_step = time.ticks_ms()
        self.enable()

    def enable(self):
        self.en_pin.value(1)

    def disable(self):
        self.en_pin.value(0)

    def stop(self):
        self.moving = False

    def move(self, dir, speed):
        self.dir = dir
        self.dir_pin.value(dir)
        self.speed = speed
        self.time_between_steps = 1000 * self.speed * self.steps_per_rev / 60
        self.moving = True
        self.time_next_step = time.ticks_ms()

    def update(self):
        if self.moving:
            if time.ticks_diff(time.ticks_ms(), self.time_next_step) > 0:
                self.step_pin.value(1)
                time.sleep_us(3)
                self.step_pin.value(0)
                self.time_next_step = time.ticks_add(time.ticks_ms(), self.time_between_steps)


class DCMotor:
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
motorM = DCMotor(0, 10, 11, 12)
motorS = DCMotor(1, 13, 14, 16)
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

