import time
import motor

from multiprocessing import Process

"""
global forward0, forward1, backward1, backward0
global pwm 
speed
motor 0 
motor 1

with speed spd  
motor0(forward0)
motor1(forward1)

motor0(backward0)
motor1(backward1)

busnum 
GPIO.BOARD (physical location)

"""


class MotorController(Process):
    """
    desc
    """

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
        motor.forward_with_speed(speed)

    def turn(self, angle):
        """
        Description of function
        """
        print('MotorController: wheel angle set to {0}'.format(angle))

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
                # este speed de donde sale?
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


            # no entiendo esta parte por que solo hay cmd de move y turn
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

        motor.setup()
        motor.setSpeed(20)
        while (True):
            self.rx_cmd()

            key = cv2.waitKey(1) & 0xFF

            if key == ord("q"):
                break



        # Once the loop is stopped, ensure all motors are off and wheels are turned to center
        self.move(speed=0)
        self.turn(angle=0)

    def stop(self):
        """
        Description of function
        """
        self.b_stopping = True