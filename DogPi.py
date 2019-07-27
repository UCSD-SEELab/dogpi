import time
import sys

from multiprocessing import Pipe

from ImageProcessor import ImageProcessor
from MotorController import MotorController


class DogPi:
    """
    The DogPi class...
    """

    def __init__(self, name):
        """
        Initializes a DogPi object with a given name. The
        """
        self.name = name

        # Initialize directional pipe between image processor and motor control
        conn_in, conn_out = Pipe(duplex=False)
        # duplex / one way communication


        # Initialize image processing
        self.p_vision = ImageProcessor(conn_out=conn_out)

        # Initialize motor control
        self.p_motor = MotorController(conn_in=conn_in)

    def start(self):
        """
        desc
        """
        # Check that pipe connection is valid
        if not(self.p_vision.check_conn_valid()):
            print('{0}: ERROR: ImageProcessor pipe is invalid.'.format(self.name))
            sys.exit()

        if not(self.p_motor.check_conn_valid()):
            print('{0}: ERROR: MotorController pipe is invalid.'.format(self.name))
            sys.exit()

        # Begin image processing and perform checks that camera is functional
        print('{0}: Starting vision process.'.format(self.name))
        self.p_vision.start()

        # while not(self.p_vision.check_vision_ready()):
        #     print('{0}: Waiting for vision to initialize...'.format(self.name))
        #     time.sleep(1)

        # Begin motor controller
        self.p_motor.start()

    def stop(self):
        """
        desc
        """
        # Stop the processes in a safe manner
        print('{0} Stopping processes.'.format(self.name))
        self.p_motor.stop()
        self.p_vision.stop()

        # exitcode of None means that the process is still running
        p_motor_exitcode = self.p_motor.join(timeout=10)
        if p_motor_exitcode is None:
            print('{0}: ERROR: MotorController failed to stop. Overriding termination.'.format(self.name))
            self.p_motor.terminate()
        else:
            print('{0}: MotorController stopped successfully.')

        p_vision_exitcode = self.p_vision.join(timeout=10)
        if p_vision_exitcode is None:
            print('{0}: ERROR: ImageProcessor failed to stop. Overriding termination.'.format(self.name))
            self.p_vision.terminate()
        else:
            print('{0}: ImageProcessor stopped successfully.'.format(self.name))

if __name__ == '__main__':
    """
    Test script with keyboard input to control DogPi
    """
    print('Hello and welcome to DogPi test.')
    name_in = input('What would you like to name DogPi today?\n')
    print('Creating a DogPi named {0}'.format(name_in))
    dogpi = DogPi(name=name_in)

    key_in = ' '
    while not(key_in == 'x'):
        key_in = input('s: start, x: stop\n')
        key_in = key_in.lower()
        if key_in == 's':
            dogpi.start()
        elif key_in == 'x':
            dogpi.stop()
