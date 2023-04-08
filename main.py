import machine
import time
from machine import ADC, Pin


class A4988Stepper:
    """
    Class to control the A4988 Stepper Driver. Requires pin ID's for the step pin, direction pin, and sleep pin.
    Uses ticks to control stepper movement in a non-blocking way to allow for hardware interrupts to occur
    """
    def __init__(self, mid, step_pin, dir_pin, sl_pin, steps_per_rev):
        self.id = mid
        self.step_pin = Pin(step_pin, Pin.OUT)
        self.dir_pin = Pin(dir_pin, Pin.OUT)
        self.en_pin = Pin(sl_pin, Pin.OUT)
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
        self.time_between_steps = 1
        self.moving = True
        self.time_next_step = time.ticks_ms()

    def update(self):
        if self.moving:
            if time.ticks_diff(time.ticks_ms(), self.time_next_step) > 0:
                # print("Did a step")
                self.step_pin.value(1)
                time.sleep_us(5)
                self.step_pin.value(0)
                
                self.time_next_step = time.ticks_add(time.ticks_ms(), self.time_between_steps)
                # print(time.ticks_ms(), self.time_next_step)


class DCMotor:
    """
    Class to control DC motors via the L298N DC Motor Driver. Requires the pin ID's for both direction pins and pwm pin
    Move and update methods also take into account motor position as given by both hall effect encoders (1x mode)
    """
    def __init__(self, mid, dir1, dir2, pwm_pin):
        self.id = mid
        self.dir1 = Pin(dir1, Pin.OUT)
        self.dir2 = Pin(dir2, Pin.OUT)
        self.pos = 0
        self.target_pos = 0
        self.dir = MOTOR_OFF
        self.speed = 63000
        self.pwm = machine.PWM(Pin(pwm_pin))
        self.pwm.freq(500)

        self.state = 0

    def move_to(self, target):
        """
        :param target: a negative target retracts the compressor plate, while a positive target extends it
        """
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
resistorPower = Pin(25, Pin.OUT)
fsrADC = ADC(resistorPin)

hallMAid = 19
hallMBid = 18
hallSAid = 20
hallSBid = 21
cntM = 0
cntS = 0

hallMA = Pin(hallMAid, Pin.IN)
hallMB = Pin(hallMBid, Pin.IN)
hallSA = Pin(hallSAid, Pin.IN)
hallSB = Pin(hallSBid, Pin.IN)


def encode_M(pin: Pin):
    """
    Keeps track of the motor position for motor M on interrupt by hall sensor 1
    :param pin: passes the instance of Pin that caused the interrupt
    """
    global cntM, hallMB
    if hallMB.value():
        cntM += 1
    else:
        cntM -= 1


def encode_S(pin: Pin):
    """
    Keeps track of the motor position for motor S on interrupt by hall sensor 2
    :param pin: passes the instance of Pin that caused the interrupt
    """
    global cntS, hallSB
    if hallSB.value():
        cntS += 1
    else:
        cntS -= 1


# Create interrupt handlers
hallMA.irq(trigger=Pin.IRQ_FALLING, handler=encode_M)
hallSA.irq(trigger=Pin.IRQ_FALLING, handler=encode_S)

# Define some useful constansts
FSR_THRESHOLD = (0.8 * 1024)  # Defines the force threshold at below which the channel is considered full
MOTOR_CCW = [1, 0]
MOTOR_CW = [0, 1]
MOTOR_OFF = [0, 0]
motorMdir = MOTOR_OFF
motorSdir = MOTOR_OFF
POS_START = 0

# Instantiate motor objects
motorM = DCMotor(0, 10, 11, 12)
motorS = DCMotor(1, 13, 14, 15)
stepper = A4988Stepper(0, 16, 17, 7, 200)

motorM.move_to(10000)
motorS.move_to(10000)
resistorPower.value(1)
#stepper.enable()
#stepper.move(1, 200)
cnt = 0
prev_force = 1024

# Main loop
while True:

    #print(cntM, cntS)
    raw_force = fsrADC.read_u16()
    smooth_force = 0.995 * prev_force + 0.005 * raw_force  # as the sensor data is very noisy, apply exponential smoothing
    prev_force = smooth_force
    if cnt % 100 == 0:
        print(smooth_force)
    cnt += 1

    # Run motor control loop
    motorM.update(cntM)
    motorS.update(cntS)
    stepper.update()

