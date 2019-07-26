import time

from multiprocessing import Process

class ImageProcessor(Process):
    """
    desc
    """

    def __init__(self, conn_out=None):
        """
        desc
        """
        super(ImageProcessor, self).__init__()
        self.conn_out = conn_out
        self.b_streaming = False
        self.b_stopping = False

    def set_output_conn(self, conn_out):
        """
        desc
        """
        # TODO Check if conn_out is a valid pipe
        self.conn_out = conn_out

    def check_conn_valid(self):
        """
        Returns boolean if the pipe connection is still valid
        """
        # TODO Check if conn_out pipe connection is valid
        return True

    def check_vision_ready(self):
        """
        Returns boolean for if vision system is ready for movement
        """
        # TODO Confirm that vision system is ready to stream to motor process
        return self.b_streaming

    def send_turn_cmd(self, direction):
        """
        Send a turn command to the motor control through the pipe to turn DogPi's wheels in a desired direction

        INPUT
            direction:
        """
        if self.check_conn_valid():
            msg = {'cmd':'turn', 'dir':direction}
            self.conn_out.send(msg)
            print('ImageProcessor->send_turn_cmd  {0}'.format(msg))
        else:
            # TODO Raise an error
            print('ERROR: ImageProcessor->send_turn_cmd pipe connection invalid.')

    def send_move_cmd(self, direction):
        """
        Send a move command to the motor control through the pipe to move DogPi forwards, backwards, or stop

        INPUT
            direction:
        """
        if self.check_conn_valid():
            msg = {'cmd':'move', 'dir':direction}
            self.conn_out.send(msg)
            print('ImageProcessor->send_move_cmd  {0}'.format(msg))
        else:
            # TODO Raise an error
            print('ERROR: ImageProcessor->send_move_cmd pipe connection invalid.')

    def run(self):
        """
        desc
        """

        # TODO Put in functionality for video streaming and processing here!
        time.sleep(5)

        # TODO Once we know that the camera is streaming properly...
        self.b_streaming = True

        while not (self.b_stopping):
            time.sleep(2)
            self.send_turn_cmd(-20)
            time.sleep(2)
            self.send_move_cmd(1)
            time.sleep(2)
            self.send_turn_cmd(20)
            time.sleep(2)
            self.send_move_cmd(-1)

    def stop(self):
        """
        Description of function
        """
        self.b_stopping = True


