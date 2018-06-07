#!/usr/bin/env python

import socket
import rospy
import sys
import time
import argparse
import json
import base64
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from ros_websocket_bridge.srv import *

class WebSocketClient():

    def __init__(self,host='192.168.1.118',port=5000):
        self.socket= socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.settimeout(30.0)
        self.host = host #"192.168.1.118" # needs to be in quote
        self.port = port
        #self.s_client = rospy.Service('detectron', Detectron, self.handle_detectron_request)
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
        buf = []
        try:
            buf = self.socket.recv(4096)
            #buf += s.recv(1)
            if len(buf) > 0:
                rospy.loginfo("Received : %s",str(buf))
            return buf
        except:
            return buf

    def close(self):
        self.send_data("close me".encode("utf-8"))
        time.sleep(1)
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

    def read_image_and_call_service(self,filepath):

        data = dict()

        cv_image = cv2.imread(filepath)

        bridge = CvBridge()

        image_message = bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")

        res = self.stairs_service(image_message)

        print "Received image width: {} height: {}".format(res.width,res.height)


    def read_and_send_image(self,filepath):

        data = dict()

        cv_image = cv2.imread(filepath)

        ret, buffer_img = cv2.imencode('.jpg', cv_image)

        #encoded = #base64.encodestring(b'data to be encoded')
        #data['bytes'] = encoded.decode('ascii')
        #img =  str(msg.data).encode("base64") #base64.b64encode(data)

        data["img"] = base64.b64encode(buffer_img)#img.encode('ascii')


        jsondata = json.dumps(data)


        if not self.send_data(jsondata):
            print "Socket error while sending data"
            rospy.signal_shutdown("Socket Error")


if __name__ == '__main__':


    rospy.init_node('ros_websocket_bridge_image_publisher')

    argparse = argparse.ArgumentParser(prog='main.py');
    argparse.add_argument("--host", type=str, help='Host address',default="localhost")
    argparse.add_argument("--port", type=int, help='Host port', default=5000)
    argparse.add_argument("--filepath", help='Image file path for reading and sending')


    args = argparse.parse_args(rospy.myargv(argv=sys.argv)[1:])


    print args.host
    print args.port
    print args.filepath

    websocketclient = WebSocketClient(host=args.host, port=args.port)

    websocketclient.stairs_service = rospy.ServiceProxy("detect_stairs",DetectStairs)

    #rospy.Subscriber("camera1_image", Image, detectronclient.image_callback)


    #if(not websocketclient.connect()):
    #    sys.exit(-1)
        #rospy.signal_shutdown("Communication Error with the Websocket Server")

    rospy.loginfo("ROS bridge started...");

    websocketclient.read_image_and_call_service(args.filepath)

    rospy.spin()
    #websocketclient.read_and_send_image(args.filepath)
    '''
    while not rospy.is_shutdown():
        data = websocketclient.listen()
        if len(data) > 0 :
            print data
            #enerothclient.parsedata(data)
    '''
    #while not rospy.is_shutdown():
        #data = detectronclient.listen()
        #if len(data) > 0 :
        #    enerothclient.parsedata(data)



    #detectronclient.close()
