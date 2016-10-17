#!/usr/bin/env python

from vulcano_web.srv import *
from sensor_msgs.msg import CompressedImage, Image
import rospy
import subprocess
import os
import json
from cv_bridge import CvBridge
import cv2
import time
import yaml
from yaml.representer import SafeRepresenter
import rospkg


logName = os.environ["LOGNAME"]
#systemLanguage = os.environ["LANG"]  #maybe LANGUAJE

recording = 0
filename_count = 0
imageCount = 0
'''
if systemLanguage == "en_US:en" or systemLanguage == "en_US.UTF-8":
    print "Language English"
    filename_string = "/home/" + logName + "/Desktop/video"
    filenameImageBase = "/home/" + logName + "/Desktop/imagen"
else:
    print "Language Spanish"
    filename_string = "/home/" + logName + "/Escritorio/video"
    filenameImageBase = "/home/" + logName + "/Escritorio/imagen"
'''
filename_string = "/home/" + logName + "/video"
filenameImageBase = "/home/" + logName + "/image"



maxAngle = 5.0
trimAngle = 0.0
minRadius = 1.0
reverseDirection = False

#filenameBase = os.getcwd() + "/src/summit_xl_web"
#JSONFilename = filenameBase + "/eliotOptions.json"
JSONFilename = os.getcwd() + "/eliotOptions.json"
controllerOptionsFilename = rospkg.RosPack().get_path('vulcano_base_hw') + "/launch" + "vulcano_base_hw.yaml"

imageMsg = CompressedImage()

bridge = CvBridge()

text= " # !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! \n # If motors_encoders == true motors speed is multiplied by motors_encoder_factor \n # If no encoders it MUST be false \n # Robot will increase his speed DANGEROUSLY if true with no encoders \n\n # Wheel diameter --> used to measure odometry \n # Gearbox reduction (normally 12.52 or 9.56) --> used to measure odometry \n # Kinematic mode (\"skid\" or \"omni\") --> if skid steering is selected you wont be able to select omni mode with the joystick \n # Motion odometry -->  if true, gyro movement will be ignored when the robot is stopped"

def start_recording(req):
    global recording
    global filename_count
    global now
    global imageCount
    global imageSubscriber

    if req.startRecording == 1 and recording == 0:
        print("Start recording")

        #now = rospy.Time.now().secs
        filename = filename_string + str(filename_count) + ".avi"
        #filename = filename_string + str(now) + ".avi"
        while os.path.isfile(filename):
            filename_count += 1
            filename = filename_string + str(filename_count) + ".avi"

        print filename
        command = "rosrun image_view video_recorder image:=/axis1 _image_transport:=compressed camera_info:=/axis1/camera_info __name:=video_recorder  _filename:=" + filename
        recording = 1
        filename_count += 1
        subprocess.call(command, shell=True)
        return RecordVideoResponse(filename)

    elif req.startRecording == 0 and recording == 1:
        print("Stop recording")
        filename = filename_string + str(filename_count-1) + ".avi"
        #filename = filename_string + str(now) + ".avi"
        command = "rosnode kill /video_recorder"
        recording = 0
        subprocess.call(command, shell=True)
        return RecordVideoResponse(filename)


    ### Save Image
    ### -----------
    elif req.startRecording == 2:
        print "save image"

        # start republish node to read std_msgs/Image
        #command = "rosrun image_transport republish compressed in:=/axis1 raw out:=/axis1 __name:=republish"
        #subprocess.call(command, shell=True)


        # wait until the republish node is started
        #time.sleep(3)

        imageSubscriber = rospy.Subscriber("/axis1", Image, imageHandler)
        #imageSubscriber = rospy.Subscriber("/axis1/compressed", CompressedImage, imageHandler)

        filenameImage = filenameImageBase + str(imageCount) + ".jpg"
        while os.path.isfile(filenameImage):
            imageCount += 1
            filenameImage = filenameImageBase + str(imageCount) + ".jpg"

        print filenameImage

        # wait to get imageMsg from /axis1 topic
        #time.sleep(3)

        #print(imageMsg.data)

        '''
        # create image raw
        imageRaw = Image()
        imageRaw.header = imageMsg.header
        imageRaw.height = 576
        imageRaw.width = 704
        imageRaw.encoding = "bgr8"
        imageRaw.is_bigendian = 0
        imageRaw.step = 0
        imageRaw.data = imageMsg.data
        '''


        # save image
        # ----------
        cv_image = bridge.imgmsg_to_cv2(imageMsg, "passthrough")
        #cv_image = bridge.imgmsg_to_cv2(imageRaw, "passthrough")
        cv2.imwrite(filenameImage, cv_image)

        # kill republish node (uses a lot of resources)
        #command = "rosnode kill /republish"
        #subprocess.call(command, shell=True)

        return RecordVideoResponse(filenameImage)



def readJSONFile():
    global maxAngle
    global trimAngle
    global minRadius
    global reverseDirection

    print("Reading JSON file")

    f = open(JSONFilename, 'r+')
    optionsJSON = json.load(f)

    maxAngle = optionsJSON["maxAngle"]
    trimAngle = optionsJSON["trimAngle"]
    minRadius = optionsJSON["minRadius"]
    reverseDirection = optionsJSON["reverseDirection"]

    #print(str(maxAngle) + " " + str(trimAngle) + " " + str(minRadius) + " " + str(reverseDirection) )

    f.close()

def newOptions(data):
    global maxAngle
    global trimAngle
    global minRadius
    global reverseDirection

    ### TODO: Change options variables
    maxAngle = data.maxAngle
    trimAngle = data.trimAngle
    minRadius = data.minRadius
    reverseDirection = data.reverseDirection
    #print(str(maxAngle) + " " + str(trimAngle) + " " + str(minRadius) + " " + str(reverseDirection) )


    ### TODO: modify JSON file
    writeJSONFile()


def writeJSONFile():
    global maxAngle
    global trimAngle
    global minRadius
    global reverseDirection

    f = open(JSONFilename, 'r')
    optionsJSON = json.load(f)
    f.close()


    optionsJSON["maxAngle"] = maxAngle
    optionsJSON["trimAngle"] = trimAngle
    optionsJSON["minRadius"] = minRadius
    optionsJSON["reverseDirection"] = reverseDirection

    f = open(JSONFilename, 'w')

    json.dump(optionsJSON, f)

    f.close()


def newJSONFile():
    global maxAngle
    global trimAngle
    global minRadius
    global reverseDirection
    print("new JSON file")

    JSONdata = {"reverseDirection": reverseDirection, "maxAngle": maxAngle, "trimAngle": trimAngle, "minRadius": minRadius}
    f = open(JSONFilename, 'w+')
    json.dump(JSONdata, f)
    f.close()


def imageHandler(data):
    global imageMsg
    global imageSubscriber

    imageMsg = data

    # this handler eats a lot of resources, so only subscribe when needed
    imageSubscriber.unregister()


class quoted_str(str): pass

def change_style(style, representer):
    def new_representer(dumper, data):
        scalar = representer(dumper, data)
        scalar.style = style
        return scalar
    return new_representer


def setControllerOptions(req):

    #print(controllerOptionsFilename)
    print("Options saved")


    f = open(controllerOptionsFilename, 'r')
    yamlControllerOptions = yaml.load(f)
    f.close()

    represent_quoted_str = change_style('"', SafeRepresenter.represent_str)
    yaml.add_representer(quoted_str, represent_quoted_str)

    yamlControllerOptions["summit_xl_controller"]["take-over"] = req.takeOver
    yamlControllerOptions["summit_xl_controller"]["kinematic_mode"] =  quoted_str(req.kinematicMode)
    yamlControllerOptions["summit_xl_controller"]["gearbox_reduction"] = round(req.gearboxReduction,2)
    yamlControllerOptions["summit_xl_controller"]["diameter_wheel"] = round(req.diameterWheel,3)
    yamlControllerOptions["summit_xl_controller"]["motion_odometry"] = quoted_str(req.motionOdometry)
    yamlControllerOptions["summit_xl_controller"]["motors_encoder"] = quoted_str(req.motorsEncoder)
    yamlControllerOptions["summit_xl_controller"]["motors_encoder_factor"] = req.motorsEncoderFactor
    yamlControllerOptions["summit_xl_controller"]["x_wam"] = quoted_str(req.xWam)


    f = open(controllerOptionsFilename, 'w')
    yaml.dump(yamlControllerOptions, f, default_flow_style = False, allow_unicode=True)

    f.write(text)
    f.close()

    return set_controller_optionsResponse(True)


def getControllerOptions(req):
    #print("handle get options")

    f = open(controllerOptionsFilename, 'r')
    yamlControllerOptions = yaml.load(f)
    f.close()

    return get_controller_optionsResponse(
    yamlControllerOptions["summit_xl_controller"]["take-over"],
    yamlControllerOptions["summit_xl_controller"]["kinematic_mode"],
    yamlControllerOptions["summit_xl_controller"]["gearbox_reduction"],
    yamlControllerOptions["summit_xl_controller"]["diameter_wheel"],
    yamlControllerOptions["summit_xl_controller"]["motion_odometry"],
    yamlControllerOptions["summit_xl_controller"]["motors_encoder"],
    yamlControllerOptions["summit_xl_controller"]["motors_encoder_factor"],
    yamlControllerOptions["summit_xl_controller"]["x_wam"])


def start_recording_server():
    rospy.init_node('start_recording_server')
    print ("Web server started.")


    ### Services
    ### ---------
    rospy.Service('start_recording', RecordVideo, start_recording)
    rospy.Service('set_controller_options', set_controller_options, setControllerOptions)
    rospy.Service('get_controller_options', get_controller_options, getControllerOptions)


    #eliotOptionsMsg = EliotOptions()

    ### Publishers
    ### --------------
    #eliotOptionsPublisher = rospy.Publisher('eliotOptions', EliotOptions)


    ### Subscribers
    ### --------------
    #rospy.Subscriber("/summit_xl_web/eliotOptionsConfig", EliotOptions, newOptions)





    '''
    if not os.path.isfile(JSONFilename):
        ###  Create new JSON file
        newJSONFile()
    else:
        readJSONFile()


    r = rospy.Rate(10)

    while not rospy.is_shutdown():


        ###  Send eliot options topic
        ###  TODO: change message info only when changes appear
        eliotOptionsMsg.maxAngle = maxAngle
        eliotOptionsMsg.trimAngle = trimAngle
        eliotOptionsMsg.minRadius = minRadius
        eliotOptionsMsg.reverseDirection = reverseDirection
        eliotOptionsPublisher.publish(eliotOptionsMsg)


        r.sleep()
    '''

    rospy.spin()



if __name__ == "__main__":
    start_recording_server()

