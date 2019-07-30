import time
from collections import deque
from imutils.video import VideoStream
import numpy as np
import argparse
import cv2
import imutils

from multiprocessing import Process

turn_right = 20
turn_left = 20
forward = 40
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
        # construct the argument parse and parse the arguments
        ap = argparse.ArgumentParser()
        ap.add_argument("-v", "--video",
                        help="path to the (optional) video file")
        ap.add_argument("-b", "--buffer", type=int, default=32,
                        help="max buffer size")
        args = vars(ap.parse_args())

        # define the lower and upper boundaries of the "green"
        # ball in the HSV color space
        greenLower = (29, 86, 6)
        greenUpper = (64, 255, 255)

        # initialize the list of tracked points, the frame counter,
        # and the coordinate deltas
        pts = deque(maxlen=args["buffer"])
        counter = 0
        (dX, dY) = (0, 0)
        direction = ""

        # if a video path was not supplied, grab the reference
        # to the webcam
        if not args.get("video", False):
            vs = VideoStream(src=0).start()

        # otherwise, grab a reference to the video file
        else:
            vs = cv2.VideoCapture(args["video"])

        # allow the camera or video file to warm up
        time.sleep(2.0)

        # keep looping
        while True:
            # grab the current frame
            frame = vs.read()

            # handle the frame from VideoCapture or VideoStream
            frame = frame[1] if args.get("video", False) else frame

            # half = frame.size/2
            half = (480 / 2)

            if frame is None:
                break

            # resize the frame, blur it, and convert it to the HSV
            # color space
            frame = imutils.resize(frame, width=600)
            blurred = cv2.GaussianBlur(frame, (11, 11), 0)
            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

            # construct a mask for the color "green", then perform
            # a series of dilations and erosions to remove any small
            # blobs left in the mask
            mask = cv2.inRange(hsv, greenLower, greenUpper)
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)

            # find contours in the mask and initialize the current
            # (x, y) center of the ball
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

                M = cv2.moments(c)

                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                # only proceed if the radius meets a minimum size
                if radius > 10:

                    size_ball = 6  # cm
                    known_distance = 40  # cm
                    width_pixels = (radius * 2)

                    # focal length
                    # focal_length = (width_pixels * known_distance) / size_ball
                    focal_length = 465.95

                    # find the distance
                    # distance = ((size_ball * focal_length) / width_pixels)
                    distance = distance_to_camara(size_ball, focal_length, width_pixels)

                    if distance > 40:
                        forward
                    else:
                        stop

                    # draw circle and centroid / update list of tracked points
                    cv2.circle(frame, (int(x), int(y)), int(radius),
                               (0, 255, 255), 2)
                    cv2.circle(frame, center, 5, (0, 0, 255), -1)

                    # call functions to turn left or right
                    if center[1] >= half:
                        turn_left
                        direction = 'Left'
                    else:
                        turn_right
                        # direction = 'Right'
                        direction = 20
            # else:
            # stop()

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

            # show the movement deltas and the direction of movement on
            # the frame
            # cv2.putText(frame, direction, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                        # 0.65, (255, 255, 255), 3)
            # cv2.putText(frame, "dx: {}, dy: {}".format(dX, dY),
                        # (10, frame.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX,
                        # 0.35, (255, 255, 255), 1)


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

        time.sleep(5)

        # TODO Once we know that the camera is streaming properly...
        self.b_streaming = True

        while not (self.b_stopping):
            time.sleep(2)
            self.send_turn_cmd(direction)
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


