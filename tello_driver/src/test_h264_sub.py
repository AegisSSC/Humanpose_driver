#!/usr/bin/env python2

# sets up the code to use python commands
import rospy
# uses the msgs from ROS to communicate between nodes
from sensor_msgs.msg import CompressedImage
# python libraries
import av
import cv2
import numpy
import threading
import traceback


# gets the video stream from the drone
class StandaloneVideoStream(object):
    # Constructor for the class
    def __init__(self):
        self.cond = threading.Condition()
        self.queue = []
        self.closed = False
    # Reads in the size of the image to stream
    def read(self, size):
        self.cond.acquire()
        try:
            if len(self.queue) == 0 and not self.closed:
                self.cond.wait(2.0)
            data = bytes()
            while 0 < len(self.queue) and len(data) + len(self.queue[0]) < size:
                data = data + self.queue[0]
                del self.queue[0]
        finally:
            self.cond.release()
        return data

    def seek(self, offset, whence):
        return -1

    def close(self):
        self.cond.acquire()
        self.queue = []
        self.closed = True
        self.cond.notifyAll()
        self.cond.release()

    def add_frame(self, buf):
        self.cond.acquire()
        self.queue.append(buf)
        self.cond.notifyAll()
        self.cond.release()


stream = StandaloneVideoStream()


def callback(msg):
    #rospy.loginfo('frame: %d bytes' % len(msg.data))
    stream.add_frame(msg.data)


def main():
    # creates the node for the listener
    rospy.init_node('h264_listener')
    # subscribes to the image stream from the tello camera
    rospy.Subscriber("/tello/image_raw/h264", CompressedImage, callback)
    # opens a container for the av stream
    container = av.open(stream)
    # verifies through ROS that the stream is opened
    rospy.loginfo('main: opened')

    #for every frame within the container, decode to separate images
    for frame in container.decode(video=0):
        image = cv2.cvtColor(numpy.array(
            frame.to_image()), cv2.COLOR_RGB2BGR)
        cv2.imshow('Frame', image)
        cv2.waitKey(1)


if __name__ == '__main__':
    try:
        main()
    except BaseException:
        traceback.print_exc()
    finally:
        stream.close()
        cv2.destroyAllWindows()
