import time
from collections import deque
from imutils.video import VideoStream
import numpy as np
import argparse
import cv2
import imutils

from multiprocessing import Process

turn_right = 0
turn_left = 0
forward = 0
stop = 0
direction =''


def distance_to_camara(size_ball, focal_length, width_pixels):
    distance = ((size_ball * focal_length) / width_pixels)
    return distance


class ImageProcessor(Process):
    """
    desc
    """

    def __init__(self, conn_out=None):
        """
        desc
        video
        default buffer
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
            msg = {'cmd':'turn', 'direction':direction}
            self.conn_out.send(msg)
            print('ImageProcessor->send_turn_cmd  {0}'.format(msg))
        else:
            # TODO Raise an error
            print('ERROR: ImageProcessor->send_turn_cmd pipe connection invalid.')

    def send_move_cmd(self, direction):
        """
        Send a move command to the motor control through the pipe to move DogPi forwards, backwards, or stop
        """
        if self.check_conn_valid():
            msg = {'cmd':'move', 'direction':direction}
            self.conn_out.send(msg)
            print('ImageProcessor->send_move_cmd  {0}'.format(msg))
        else:
            # TODO Raise an error
            print('ERROR: ImageProcessor->send_move_cmd pipe connection invalid.')


    def run(self):
        """
        desc
        """
        # define the lower and upper boundaries of the "green"
        # ball in the HSV color space
        greenLower = (29, 86, 6)
        greenUpper = (64, 255, 255)

        # initialize the list of tracked points, the frame counter,
        # and the coordinate deltas
        # pts = deque(maxlen=args["buffer"])
        counter = 0
        (dX, dY) = (0, 0)
        direction = ""


        # vs = cv2.VideoCapture(args["video"])
        vs = cv2.VideoCapture(1)
        time.sleep(2.0)

        # keep looping
        while True:
            # grab the current frame
            frame = vs.read()

            half = ((frame.shape[1]) / 2)
            if frame is None:
                break

            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # construct a mask for the color "green", then perform
            # a series of dilations and erosions to remove any small
            # blobs left in the mask

            mask = cv2.inRange(hsv, greenLower, greenUpper)
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)

            # resize the frame, blur it
            mask = imutils.resize(mask, width=600)
            blurred = cv2.GaussianBlur(mask, (11, 11), 0)

            # find contours in the mask and initialize the current
            # (x, y) center of the ball
            # find contours of blurred version
            cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)
            cnts = imutils.grab_contours(cnts)
            # print ('Contours', len(cnts))
            center = None

            # only proceed if at least one contour was found
            if len(cnts) > 0:

                # largest contour in maks - min enclosing circle and centroid
                c = max(cnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                # moment for non circular objcts
                M = cv2.moments(c)

                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                # only proceed if the radius meets a minimum size
                if radius > 10:

                    size_ball = 6  # cm
                    known_distance = 40  # cm
                    width_pixels = (radius * 2)

                    # focal length
                    # focal_length = (width_pixels * known_distance) / size_ball
                    focal_length = 1054

                    # find the distance
                    # distance = ((size_ball * focal_length) / width_pixels)
                    distance = distance_to_camara(size_ball, focal_length, width_pixels)

                    if distance > 50:
                        forward = +10
                        self.send_move_cmd(forward)
                        # direction
                    else:
                        stop = 0
                        self.send_move_cmd(stop)


                    # draw circle and centroid / update list of tracked points
                    cv2.circle(frame, (int(x), int(y)), int(radius),
                               (0, 255, 255), 2)
                    cv2.circle(frame, center, 5, (0, 0, 255), -1)

                    pixels_from_center = half - center[1]
                    angle = np.arctan((pixels_from_center * size_ball) / (width_pixels * distance))

                    degrees = (angle * (180/np.pi))

                    self.send_turn_cmd(degrees)

            # loop over the set of tracked points
            for i in np.arange(1, len(pts)):
                #  if tracked points = None then ignore
                if pts[i - 1] is None or pts[i] is None:
                    continue

                # check enough points in buffer
                if counter >= 10 and i == 1 and pts[-10] is not None:

                    # compute the difference between the x and y
                    # coordinates and re-initialize the direction
                    # text variables
                    dX = pts[-10][0] - pts[i][0]
                    dY = pts[-10][1] - pts[i][1]
                    (dirX, dirY) = ("", "")

                    # significant movement in x-direction
                    if np.abs(dX) > 20:

                        if np.sign(dX) == 1:
                            dirX = "Left"

                        else:
                            dirX = "Right"

                    # ensure there is significant movement in the
                    # y-direction
                    if np.abs(dY) > 20:
                        dirY = "Up" if np.sign(dY) == 1 else "Down"

                    # handle when both directions are non-empty
                    if dirX != "" and dirY != "":
                        direction = "{}-{}".format(dirY, dirX)

                    # otherwise, only one direction is non-empty
                    else:
                        direction = dirX if dirX != "" else dirY

                # thickness of the line and draw connecting line
                thickness = int(np.sqrt(args["buffer"] / float(i + 1)) * 2.5)
                cv2.line(frame, pts[i - 1], pts[i], (0, 0, 255), thickness)

            # if the 'q' key is pressed, stop the loop
            cv2.imshow('Frame', frame)
            counter += 1
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break

        # if we are not using a video file, stop the camera video stream
        if not args.get("video", False):
            vs.stop()

        # otherwise, release the camera
        else:
            vs.release()

        cv2.destroyAllWindows()

        time.sleep(5)

        # TODO Once we know that the camera is streaming properly...
        self.b_streaming = True

        while not (self.b_stopping):
            pass
            # time.sleep(2)
            # self.send_turn_cmd(turn_left)
            # time.sleep(2)
            # self.send_move_cmd(1)
            # time.sleep(2)
            # self.send_turn_cmd(turn_right)
            # time.sleep(2)
            # self.send_move_cmd(-1)

    def stop(self):
        """
        Description of function
        """
        self.b_stopping = True


