#!/usr/bin/env python

import socket
import rospy
import sys
import time
import argparse
import json
import base64
from ros_websocket_bridge.srv import *
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from PIL import Image as Im
import io

class StairsClient():

    def __init__(self,host='192.168.1.118',port=5000):
        self.socket= socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.settimeout(30.0)
        self.host = host #"192.168.1.118" # needs to be in quote
        self.port = port
        self.s_client = rospy.Service('detect_stairs', DetectStairs, self.handle_detect_stairs_request)
        self.bridge = CvBridge()
        self.ji = 0

    def connect(self):
        try:
           self.socket.connect((self.host, self.port))
        except Exception as ex:
           print str(ex)
           return False

        time.sleep(1)
        #self.socket.send("client_type;{}".format(self.client_type).encode("utf-8"))

        return True

    def listen(self):
        buf = b''
        try:
            buf = self.socket.recv(4096)
            #buf += s.recv(1)
            #if len(buf) > 0:
                #rospy.loginfo("Received : %s",str(buf))
            return buf
        except:
            return buf

    def close(self):
        #self.send_data("close me".encode("utf-8"))
        #time.sleep(1)
        self.socket.close()

    def send_data(self, data):
        try:
            self.socket.sendall(data)
            time.sleep(1)
            return True
        except Exception as ex:
            print str(ex)
            return False
            #rospy.signal_shutdown("Socket connection failure")

    # Take in base64 string and return cv image
    def stringToRGB(self,base64_string):
        dataArray = np.frombuffer(base64.decodestring(base64_string), np.float32)
        #print dataArray.shape
        #imgdata = base64.b64decode(str(base64_string))
        #im = np.fromstring(base64_string)#.astype(np.float16)
        im = dataArray.reshape(544,960)
        #print im.shape
        #print np.where(im > 0.5)
        image = Im.fromarray(im)
        filename = "deneme"
        filename +=".tiff"
        print(filename)

        image.save(filename)
        #image =Im.open(io.BytesIO(imgdata))
        return dataArray#cv2.cvtColor(np.array(image), cv2.COLOR_BGR2RGB)



    def handle_detect_stairs_request(self,req):

        data = dict()
        try:
            cv_image = self.bridge.imgmsg_to_cv2(req.image, "bgr8")
        except CvBridgeError as e:
            print(e)
            return

        ret, buffer_img = cv2.imencode('.jpg', cv_image)

        #img = base64.b64encode(req.image.data)

        data['img'] = base64.b64encode(buffer_img)

        if not self.send_data(json.dumps(data)):
            return DetectStairsResponse([],0,0)
        incoming_data=b''

        while 1:
            try:
                incoming_data += self.listen()
                jsonresp = json.loads(incoming_data)

                data_array = self.stringToRGB(jsonresp[1])
                image_sizes = jsonresp[2].split(",")
                height= int(image_sizes[0][1:])
                width= int(image_sizes[1][:-1])
                return DetectStairsResponse(data_array,height,width)
            #image_size = jsonresp[2]
            #print type(image_size)


                #print img
            except Exception as ex:
                print ex
                print "keep trying"

        return DetectStairsResponse([],0,0)


if __name__ == '__main__':


    rospy.init_node('stairs_detector_bridge')

    argparse = argparse.ArgumentParser(prog='stairs_detector.py');
    argparse.add_argument("--host", type=str, help='Host address',default="localhost")
    argparse.add_argument("--port", type=int, help='Host port', default=5000)
    #argparse.add_argument("--camera_topic", type=str, help='camera topic to listen', default="kinect2/qhd/image_color")


    args = argparse.parse_args(rospy.myargv(argv=sys.argv)[1:])


    print args.host
    print args.port
    stairsclient = StairsClient(host=args.host, port=args.port)

    #rospy.Subscriber(args.camera_topic, Image, detectronclient.image_callback)


    if(not stairsclient.connect()):
        print "Cannot connect to remote server"
        sys.exit(-1)
        #rospy.signal_shutdown("Communication Error with the Server")

    rospy.loginfo("Stairs Detector ROS bridge started...");

    rospy.spin()

    stairsclient.socket.close()
    #while not rospy.is_shutdown():



            #print len(data.split(","))

            #enerothclient.parsedata(data)

    #while not rospy.is_shutdown():
        #data = detectronclient.listen()
        #if len(data) > 0 :
        #    enerothclient.parsedata(data)



    #detectronclient.close()
