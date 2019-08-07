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

class MotorController(Process):
    """
    desc
    """
    # car direction 

    def Map(x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def setup(busnum=None):
        global leftPWM, rightPWM, homePWM, pwm
        leftPWM = 150
        homePWM = 200
        rightPWM = 255
        try:
            for line in open('config'):
                if line.startswith('leftPWM'):
                    leftPWM = int(line.split('='))
        except:
            print('config error')
        if busnum == None:
            pwm = servo.PWM()  # Initialize the servo controller.
        else:
            pwm = servo.PWM(bus_number=busnum)  # Initialize the servo controller.
        pwm.frequency = 30  # changed from 60 to 30

    def turn_left():
        global leftPWM
        #pwm.write(0, 0, leftPWM)
        print ('Turn Left')# CH0

    def turn_right():
        global rightPWM
        #pwm.write(0, 0, rightPWM)
        print ('Turn Right')

    def home():
        global homePWM
        pwm.write(0, 0, homePWM)

    # ===========================================================================
    # Adjust the duty cycle of the square waves output from channel 4 and 5 of
    # the servo driver IC, so as to control the speed of the car.
    # ===========================================================================

    def setSpeed(self, speed):
        # speed *= 10  # numero entre 0  y 100
        print('speed is: {0}'.format(speed))
        pwm.write(EN_M0, 0, speed)
        pwm.write(EN_M1, 0, speed)

    def setupM(self, busnum=None):
        global forward0, forward1, backward1, backward0
        global pwm
        if busnum == None:
            pwm = p.PWM()  # Initialize the servo controller.pytho
        else:
            pwm = p.PWM(bus_number=busnum)  # Initialize the servo controller.

        pwm.frequency = 60  # 60 times a second is swtiching
        forward0 = 'True'
        forward1 = 'True'
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)  # Number GPIOs by its physical location
        try:
            for line in open("config"):
                if line[0:8] == "forward0":
                    forward0 = line[11:-1]
                if line[0:8] == "forward1":
                    forward1 = line[11:-1]
        except:
            pass
        if forward0 == 'True':
            backward0 = 'False'
        elif forward0 == 'False':
            backward0 = 'True'
        if forward1 == 'True':
            backward1 = 'False'
        elif forward1 == 'False':
            backward1 = 'True'
        for pin in pins:
            GPIO.setupM(pin, GPIO.OUT)  # Set all pins' mode as output

    def motor0(x):
        """
        if x == 'True':
            GPIO.output(Motor0_A, GPIO.LOW)
            GPIO.output(Motor0_B, GPIO.HIGH)
        elif x == 'False':
            GPIO.output(Motor0_A, GPIO.HIGH)
            GPIO.output(Motor0_B, GPIO.LOW)
        else:
            print('Config Error')
        """
        print('Motor 0- ', 'forward0-' , forward0)

    def motor1(x):
        """
        if x == 'True':
            GPIO.output(Motor1_A, GPIO.LOW)
            GPIO.output(Motor1_B, GPIO.HIGH)
        elif x == 'False':
            GPIO.output(Motor1_A, GPIO.HIGH)
            GPIO.output(Motor1_B, GPIO.LOW)
        """
        print('Motor 1- ', 'forward1- ', forward1)

    def __init__(self, conn_in=None):
        """
        desc
        """
        super(MotorController, self).__init__()
        self.conn_in = conn_in
        self.b_stopping = False

    def set_input_conn(self, conn_in):
        """
        desc
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
        Description of function
        """
        print('MotorController: movement set to {0}'.format(speed))
        # It's just going to make it go forward,  no speed
        # self.setSpeed(speed)
        motor0(forward0)
        motor1(forward1)

    def turn(self, angle):
        """
        Description of function
        """
        # print('MotorController: wheel angle set to {0}'.format(angle))
        if angle = 'Left':
            turn_left()
        else:
            turn_right()


    def rx_cmd(self):
        """
        desc
        """
        # Check if data is available (check at 50 Hz max)
        b_dataready = self.conn_in.poll(timeout=0.02)

        if not(b_dataready):
            return

        # Receive message
        # msg = {'cmd':'move', 'dir':direction}
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
                self.turn(angle=45)

            elif msg_in['cmd'] == 'turn_right':
                self.turn(angle=-45)

    def run(self):
        """
        desc
        """
            #no estoy segura de que el loop debería de ser así
            # como decirle que pare, cuando va a parar?
            # deberia de agregar una función que diga stop?
            while(True):
                self.rx_cmd()
            if not:
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