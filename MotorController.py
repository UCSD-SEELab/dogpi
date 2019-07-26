import time

from multiprocessing import Process

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
        while not(self.b_stopping):
            self.rx_cmd()

        # Once the loop is stopped, ensure all motors are off and wheels are turned to center
        self.move(speed=0)
        self.turn(angle=0)

    def stop(self):
        """
        Description of function
        """
        self.b_stopping = True