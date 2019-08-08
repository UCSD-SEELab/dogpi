import time
import RPi.GPIO as GPIO
import PCA9685 as p
import time
import PCA9685 as servo

from multiprocessing import Process

# ===========================================================================
# Raspberry Pi pin11, 12, 13 and 15 to realize the clockwise/counterclockwise
# rotation and forward and backward movements
# ===========================================================================
Motor0_A = 11  # pin11
Motor0_B = 12  # pin12
Motor1_A = 13  # pin13
Motor1_B = 15  # pin15

# ======================================    =====================================
# Set channel 4 and 5 of the servo driver IC to generate PWM, thus
# controlling the speed of the car
# ===========================================================================
EN_M0 = 4  # servo driver IC CH4
EN_M1 = 5  # servo driver IC CH5


pins = [Motor0_A, Motor0_B, Motor1_A, Motor1_B]


def Map( x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


class MotorController(Process):
    """
    desc
    """
    def __init__(self, conn_in=None):
        """
        Initialize Motor Controller
        """
        super(MotorController, self).__init__()
        self.conn_in = conn_in
        self.b_stopping = False

    # car direction
    # one setup for mottor and direction
    # pwm se repite!
    def setup(self, busnum=None):

        self.leftPWM = 150
        self.homePWM = 200
        self.rightPWM = 255
        try:
            for line in open('config'):
                if line.startswith('leftPWM'):
                    self.leftPWM = int(line.split('=')[-1])
                if line.startswith('rightPWM'):
                    self.rightPWM = int(line.split('=')[-1])
                if line.startswith('homePWM'):
                    self.homePWM = int(line.split('=')[-1])

        except:
            print('config error')

        if busnum == None:
            self.pwm = servo.PWM()  # Initialize the servo controller.
        else:
            self.pwm = servo.PWM(bus_number=busnum)  # Initialize the servo controller.
            self.pwm.frequency = 30  # changed from 60 to 30

    def turn_left(self):
        self.turn(angle = 30)

    def turn_right(self):
        self.turn(angle = -30)

    def home(self):
        self.turn(angle = 0)

    # ===========================================================================
    # Adjust the duty cycle of the square waves output from channel 4 and 5 of
    # the servo driver IC, so as to control the speed of the car.
    # ===========================================================================

    def setSpeed(self, speed):
        # speed *= 10  # numero entre 0  y 100
        print('speed is: {0}'.format(speed))
        self.pwm.write(EN_M0, 0, speed)
        self.pwm.write(EN_M1, 0, speed)

    def setupM(self, busnum=None):

        if busnum == None:
            self.pwm = p.PWM()  # Initialize the servo controller.pytho
        else:
            self.pwm = p.PWM(bus_number=busnum)  # Initialize the servo controller.

        self.pwm.frequency = 60  # 60 times a second is swtiching
        self.forward0 = 'True'
        self.forward1 = 'True'
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)  # Number GPIOs by its physical location
        try:
            for line in open("config"):
                if line[0:8] == "forward0":
                    self.forward0 = line[11:-1]
                if line[0:8] == "forward1":
                    self.forward1 = line[11:-1]
        except:
            pass
        if self.forward0 == 'True':
            self.backward0 = 'False'
        elif self.forward0 == 'False':
            self.backward0 = 'True'
        if self.forward1 == 'True':
            backward1 = 'False'
        elif self.forward1 == 'False':
            backward1 = 'True'
        for pin in pins:
            GPIO.setup(pin, GPIO.OUT)  # Set all pins' mode as output

    def motor0(self,x):

        if x == 'True':
            GPIO.output(Motor0_A, GPIO.LOW)
            GPIO.output(Motor0_B, GPIO.HIGH)
        elif x == 'False':
            GPIO.output(Motor0_A, GPIO.HIGH)
            GPIO.output(Motor0_B, GPIO.LOW)
        else:
            print('Config Error')

        print('Motor 0- ', 'forward0- ' , self.forward0)

    def motor1(self,x):

        if x == 'True':
            GPIO.output(Motor1_A, GPIO.LOW)
            GPIO.output(Motor1_B, GPIO.HIGH)
        elif x == 'False':
            GPIO.output(Motor1_A, GPIO.HIGH)
            GPIO.output(Motor1_B, GPIO.LOW)

        print('Motor 1- ', 'forward1- ', self.forward1)


    def set_input_conn(self, conn_in):
        """
        Setup for input connection
        Assign variable conn_in
        """
        # TODO Check if conn_in is a valid pipe
        self.conn_in = conn_in

    def check_conn_valid(self):
        """
        Returns if the pipe connection is still valid
        """
        # TODO Check if conn_in pipe connection is valid
        return True

    def move(self, speed):
        """
        Movement of the DogPi forward or backwards with constant speed
        """
        print('MotorController: movement set to {0}'.format(speed))
        # It's just going to make it go forward,  no speed
        # self.setSpeed(speed)

        if speed > 0:
            self.motor0(self.forward0)
            self.motor1(self.forward1)
        elif speed < 0:
            self.motor0(self.backward0)
            self.motor1(self.backward1)


    def turn(self, angle):
        """
        Turning of the DogPi depending on it's position on the frame
        """
        print('MotorController: wheel angle set to {0}'.format(angle))

        if angle > 15:
            self.pwm.write(0, 0, self.leftPWM)
        elif angle < -15 :
            self.pwm.write(0,0, self.rightPWM)
        else:
            self.pwm.write(0,0, self.homePWM)


    def rx_cmd(self):
        """
        Receive comand
        """
        # Check if data is available (check at 50 Hz max)
        b_dataready = self.conn_in.poll(timeout=0.02)

        if not(b_dataready):
            return

        # Receive message
        msg_in = self.conn_in.recv()
        print('MotorController->rx_cmd  {0}'.format(msg_in))

        # Parse the message
        if 'cmd' in msg_in:
            if (msg_in['cmd'] == 'move') and ('direction' in msg_in):
                speed = int(msg_in['direction'])
                if speed:
                    self.move(speed)
                else:
                    print('ERROR: MotorController->rx_cmd  Invalid speed value')
            elif (msg_in['cmd'] == 'turn') and ('direction' in msg_in):
                angle = int(msg_in['direction'])
                if angle:
                    self.turn(angle=angle)
                else:
                    print('ERROR: MotorController->rx_cmd  Invalid angle value')


            elif msg_in['cmd'] == 'stop':
                self.move(speed=0)

            elif msg_in['cmd'] == 'forward':
                self.move(speed=1)

            elif msg_in['cmd'] == 'backward':
                self.move(speed=-1)

            elif msg_in['cmd'] == 'turn_left':
                self.turn_left()

            elif msg_in['cmd'] == 'turn_right':
                self.turn_right()

    def run(self):

        """
        desc
        """
        self.setupM()
        self.setup()

        while(True):
            self.rx_cmd()
        else:
            # Once the loop is stopped, ensure all motors are off and wheels are turned to center
            self.move(speed=0)
            self.turn(angle=0)

    def stop(self):
        """
        Description of function
        """
        for pin in pins:
            GPIO.output(pin, GPIO.LOW)

        self.b_stopping = True