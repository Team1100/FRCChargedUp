#!/usr/bin/env python3
from cscore import CameraServer
from networktables import NetworkTablesInstance
from networktables import NetworkTables
from apriltag import apriltag

import time
import cv2
import json
import numpy as np
import math
import threading

class CameraView(object):
    def __init__(self, camera, vertFOV, horizFOV, elevationOfTarget, elevationOfCamera, angleFromHoriz):
        self.camera = camera
        self.width = self.camera['width']
        self.height = self.camera['height']
        self.vertFOV = vertFOV
        self.horizFOV = horizFOV
        self.elevationOfTarget = elevationOfTarget
        self.elevationOfCamera = elevationOfCamera
        self.angleFromHoriz = angleFromHoriz
        self.cameraCenter = self.width/2
        self.radiusFromAxisOfRotation = 14/12 # measured in feet (distance from camera to the axis of rotation of the robot)

# A class used to describe AprilTag targets. 
class AprilTagTarget(object):
    def __init__(self, camera, coor, id):
        self.id = id
        self.offset = coor[0] - camera.cameraCenter
        self.normalizedY = (coor[1] - camera.height/2)/(camera.height/2) * -1
        self.normalizedX = (coor[0] - camera.width/2)/(camera.width/2)
        self.pitch = (self.normalizedY/2) * camera.vertFOV
        self.yaw = (self.normalizedX/2) * camera.horizFOV
        #(height of target [feet] - height of camera [feet])/tan(pitch [degrees] + angle of camera [degrees])
        self.distanceToTarget = (camera.elevationOfTarget - camera.elevationOfCamera) / math.tan(math.radians(self.pitch + camera.angleFromHoriz))

    def calculateAdjustedYaw(self, radiusFromAxisOfRotation):
        return self.yaw * (radiusFromAxisOfRotation/(self.distanceToTarget+radiusFromAxisOfRotation))
        
# A class used to describe an object detected by its color (hsv) values.
class TapeTarget(object):
    def __init__(self, imageResult, approx, tapeTargetDetected, camera, areaR):
        self.tapeTargetDetected = tapeTargetDetected
        self.imageResult = imageResult
        if self.tapeTargetDetected:
            self.x, self.y, self.w, self.h, = cv2.boundingRect(approx)
        else:
            self.x, self.y, self.w, self.h, = 1, 1, 1, 1 
        self.boundingArea = self.w * self.h
        self.normalizedY = (self.y - camera.height/2)/(camera.height/2) * -1
        self.normalizedX = (self.x - camera.width/2)/(camera.width/2)
        self.pitch = (self.normalizedY/2) * camera.vertFOV
        self.yaw = (self.normalizedX/2) * camera.horizFOV
        self.offset = self.x + self.w/2 - camera.cameraCenter
        self.aspectRatio = self.w/self.h
        self.areaRatio = areaR
        self.boundingArea = self.w * self.h
        #(height of target [feet] - height of camera [feet])/tan(pitch [degrees] + angle of camera [degrees])
        self.distanceToTarget = (camera.elevationOfTarget - camera.elevationOfCamera) / math.tan(math.radians(self.pitch + camera.angleFromHoriz))

    def ordered_cluster(self, data, max_diff):
        current_group = ()
        for item in data:
            test_group = current_group + (item, )
            test_group_mean = mean(test_group)
            if all((abs(test_group_mean - test_item) < max_diff for test_item in test_group)):
                current_group = test_group
            else:
                yield current_group
                current_group = (item, )
        if current_group:
            yield current_group

    def drawRectangle(self):
        # Draw rectangle on the Image
        cv2.rectangle(self.imageResult, (self.x,self.y),(self.x+self.w,self.y+self.h),(0,255,0),3)

class VisionApplication(object):
    def __init__(self):
        self.TITLE = "apriltag_view"
        self.TAG = "tag16h5"
        self.MIN_MARGIN = 10
        self.FONT = cv2.FONT_HERSHEY_SIMPLEX
        self.RED = 0,0,255
        self.detector = apriltag(self.TAG)

        self.imgResult = None
        self.team = None

        self.tapeTargetDetected = False

        self.distanceFromTarget = 0
        self.vision_nt = None 
        self.processingForAprilTags = False
        self.processingForColor = True
        self.usingComputerIP = False 

        # Initialize configuration
        self.config = self.readConfig()
        self.team = self.config["team"]

        self.imgResult = None
        self.mask = None

        self.cameraInUse = 1

        # Set Number of Cameras
        ##### ****************** ##
        self.numberOfCameras = 2 ##
        ##### ****************** ##

        self.aprilTagTargetID = 1
        self.detectionMode = 0

        self.hueMin = 76
        self.hueMax = 127
        self.satMin = 53
        self.satMax = 212
        self.valMin = 89
        self.valMax = 255
       
        self.myColors = [[self.hueMin,self.satMin,self.valMin,self.hueMax,self.satMax,self.valMax]]

        self.areaRatio = 0 # this is the areaRatio of every contour that is seen by the camera
        self.largestAreaRatio = 0 # this is the areaRatio of the target once it has been isolated
        self.aspectRatio = 0 # this is the aspectRatio of every contour that is seen by the camera (width/height)
        self.largestAspectRatio = 0 # this is the aspectRatio fo the target once it has been isolated

        self.colorDetectConstants = []
        self.garea = 150
        self.contours = None
        self.targets = []
        self.tapeTargetList = []

        #TODO: Fill out values below if distance calculation is desired. The first value is for camera #1, the second is for camera #2. If no second camera exists, set all the second values to 1.
        # Distance Calculation Constants
        #Vertical Field of View (Degrees)
        vertFOV = [48.94175846, 1]

        #Horizontal Field of View (Degrees)
        horizFOV = [134.3449419, 1]

        #Height of the target off the ground (feet)
        elevationOfTarget = [1.5, 1]

        #Height of the Camera off the ground (feet)
        elevationOfCamera = [0.9, 1] 

        #Angle the camera makes relative to the horizontal (degrees)
        angleFromHoriz = [30, 1]


        self.camera = CameraView(self.config['cameras'][0], vertFOV[0], horizFOV[0], elevationOfTarget[0], elevationOfCamera[0], angleFromHoriz[0])
        if self.numberOfCameras == 2:
            self.camera2 = CameraView(self.config['cameras'][1], vertFOV[1], horizFOV[1], elevationOfTarget[1], elevationOfCamera[1], angleFromHoriz[1])

        self.currentCamera = self.camera
        # Initialize Camera Server
        self.initializeCameraServer()

        # Initialize NetworkTables Client
        self.initializeNetworkTables()

    def readConfig(self):
        config = None
        with open('/boot/frc.json') as fp:
            config = json.load(fp)
        return config

    def initializeCameraServer(self):
        cserver = CameraServer.getInstance()
        # Starting Automatic Capture
        camera1 = cserver.startAutomaticCapture(name="cam1", path='/dev/v4l/by-id/usb-Ingenic_Semiconductor_CO.__LTD._HD_Web_Camera_Ucamera001-video-index0')
        camera1.setResolution(self.camera.width,self.camera.height)

        if self.numberOfCameras == 2:
            camera2 = cserver.startAutomaticCapture(name="cam2", path='/dev/v4l/by-id/usb-046d_HD_Pro_Webcam_C920_9B6E80AF-video-index0')
            camera2.setResolution(self.camera2.width,self.camera2.height)

        


        self.cvsrc = cserver.putVideo("visionCam", self.camera.width,self.camera.height)
        self.cvmask = cserver.putVideo("maskCam", self.camera.width, self.camera.height)
        
        self.sink = cserver.getVideo(name="cam1")
        if self.numberOfCameras == 2:
            self.sink2 = cserver.getVideo(name="cam2")

    def initializeNetworkTables(self):
        # Table for vision output information
        ntinst = NetworkTablesInstance.getDefault()
        
        cond = threading.Condition()
        notified = [False]

        def connectionListener(connected, info):
            print(info, '; Connected=%s' % connected)
            with cond:
                notified[0] = True
                cond.notify()

        # Decide whether to start using team number or IP address
        if self.usingComputerIP:
            ip = '192.168.102.168' #ip of the computer
            # Ex: ip = '192.168.132.5'
            print("Setting up NetworkTables client for team {} at {}".format(self.team,ip))
            ntinst.startClient(ip)
        else:
            ntinst.startClientTeam(self.team)
            print("Connected to robot")

        ntinst.addConnectionListener(connectionListener, immediateNotify=True)

        with cond:
            print("Waiting")
            if not notified[0]:
                cond.wait()

        print("Connected!")
        
        self.vision_nt = ntinst.getTable('Shuffleboard/Vision')

    def putMaskingValues(self):
        self.vision_nt.putNumber('hueMin',self.hueMin)
        self.vision_nt.putNumber('hueMax',self.hueMax)
        self.vision_nt.putNumber('satMin',self.satMin)
        self.vision_nt.putNumber('satMax',self.satMax)
        self.vision_nt.putNumber('valMin',self.valMin)
        self.vision_nt.putNumber('valMax',self.valMax)

    def getMaskingValues(self):
        self.hueMin = int(self.vision_nt.getNumber('hueMin',self.hueMin))
        self.hueMax = int(self.vision_nt.getNumber('hueMax',self.hueMax))
        self.satMin = int(self.vision_nt.getNumber('satMin',self.satMin))
        self.satMax = int(self.vision_nt.getNumber('satMax',self.satMax))
        self.valMin = int(self.vision_nt.getNumber('valMin',self.valMin))
        self.valMax = int(self.vision_nt.getNumber('valMax',self.valMax))
        self.myColors = [[self.hueMin,self.satMin,self.valMin,self.hueMax,self.satMax,self.valMax]]

    def getAprilTagTargetID(self):
        self.aprilTagTargetID = self.vision_nt.getNumber('aprilTagTargetID',1)

    def getColorDetectConst(self):
        self.colorDetectConstants = self.vision_nt.getNumberArray('colorDetectConst',[1, 1, 1, 1, 90, 100, -1, 75])

    def getDetectionMode(self):
        self.detectionMode = self.vision_nt.getNumber('detectionMode',0)
        if self.detectionMode == 0:
            self.processingForColor = False
            self.processingForAprilTags = False
        elif self.detectionMode == 1:
            self.processingForColor = True
            self.processingForAprilTags = False
        elif self.detectionMode == 2:
            self.processingForColor = False
            self.processingForAprilTags = True
    
    def getCameraInUse(self):
        self.cameraInUse = self.vision_nt.getNumber('cameraInUse',0)
        if self.cameraInUse == 1:
            self.currentCamera = self.camera
        if self.cameraInUse == 2 and self.numberOfCameras == 2:
            self.currentCamera = self.camera2

    def getImageMask(self, img, myColors):
        imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)  
        lower = np.array(myColors[0][0:3])
        upper = np.array(myColors[0][3:6])
        mask = cv2.inRange(imgHSV, lower, upper)
        return mask


    def getContours(self, img):
        tempImg, contours, hierarchy = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        return contours

    def isolateTarget(self, contours):
        idealAreaRatio = self.colorDetectConstants[0] # this is the ideal ratio for the area ratio value
        areaTolerance = self.colorDetectConstants[1] # this is the tolerance for finding the target with the right aspect ratio
        
        idealAspectRatio = self.colorDetectConstants[2] # this is the ideal aspect ratio based off of the diagram but can be changed as needed.
        aspectTolerance = self.colorDetectConstants[3]

        

        # idealYCoor is the Y coordinate where the target should usually be
        if self.colorDetectConstants[4] == -1:
            idealYCoor = self.currentCamera.height/2
        else:
            idealYCoor = self.colorDetectConstants[4]
        # yCoorTolerance is added and subtracted from the ideal coordinate to create a range on the y axis where the target should be.
        # Any target detected outside that range is ignored
        yCoorTolerance = self.colorDetectConstants[5]

        # idealXCoor is the X coordinate where the target should usually be
        if self.colorDetectConstants[6] == -1:
            idealXCoor = self.currentCamera.width/2
        else:
            idealXCoor = self.colorDetectConstants[6]
        # xCoorTolerance is added and subtracted from the ideal coordinate to create a range on the x axis where the target should be.
        # Any target detected outside that range is ignored
        xCoorTolerance = self.colorDetectConstants[7]

        # start off with a large tolerance, and if the ideal ratio is correct, lower the tolerance as needed. 
        self.targets = [] 
        
        self.tapeTargetDetected = False
        if len(contours) > 0:
            largest = contours[0]
            area = 0
            for contour in contours:
                contourArea = cv2.contourArea(contour) #area of the particle
                x, y, w, h, = cv2.boundingRect(contour)
                boundingArea = w * h
                # ignores targets that are too small
                if (boundingArea < self.colorDetectConstants[8]):
                    continue
                #ignores targets that are outside a predetermined range
                if not ((y < (idealYCoor + yCoorTolerance)) and (y > (idealYCoor - yCoorTolerance))):
                    print("Target Outside of Y-Coor Range")
                    continue
                if not ((x < (idealXCoor + xCoorTolerance)) and (x > (idealXCoor - xCoorTolerance))):
                    print("Target Outside of X-Coor Range")
                    continue
                self.areaRatio = contourArea/boundingArea
                self.aspectRatio = w/h
                if self.areaRatio > idealAreaRatio - areaTolerance and self.areaRatio < idealAreaRatio + areaTolerance: # if the targets are within the right area ratio range, they is possibly the correct target
                    if self.aspectRatio > idealAspectRatio - aspectTolerance and self.aspectRatio < idealAspectRatio + aspectTolerance: # if the target is within the correct aspect ratio range aswell, it is definitely the right target
                        largest = contour
                        self.tapeTargetDetected = True
                        self.garea = boundingArea
                        self.targets.append(contour)
                        # Draw the contours
                        cv2.drawContours(self.imgResult, largest, -1, (255,0,0), 3)

    def drawBoundingBox(self):
        if self.tapeTargetDetected:
            for target in self.targets:
                try:
                    peri = cv2.arcLength(target, True)
                except:
                    print("CV2 Error")
                    continue
                approx = cv2.approxPolyDP(target, 0.02 * peri, True)
                x, y, w, h, = cv2.boundingRect(target)
                boundingArea = w * h
                contourArea = cv2.contourArea(target)
                self.tapeTargetList.append(TapeTarget(self.imgResult, approx, self.tapeTargetDetected, self.camera,(contourArea/boundingArea)))
        else:
            approx = None

    def processImgForTape(self, input_img):
        self.getMaskingValues()
        self.mask = self.getImageMask(input_img,self.myColors)
        self.contours = self.getContours(self.mask)
        self.isolateTarget(self.contours)
        self.drawBoundingBox()

    def runApplication(self):
        input_img1 = np.zeros(shape=(self.camera.height,self.camera.width,3),dtype=np.uint8)
        targetDetTol = 1.0 
        t1 = 0
        t2 = 0
        self.getColorDetectConst()
        while True:
            self.getDetectionMode()
            self.getCameraInUse()
            if self.cameraInUse == 1 or self.numberOfCameras == 1:
                frame_time1, input_img1 = self.sink.grabFrame(input_img1)
                input_img1 = cv2.resize(input_img1, (self.camera.width,self.camera.height), interpolation = cv2.INTER_AREA)
            else:
                frame_time1, input_img1 = self.sink2.grabFrame(input_img1)
                input_img1 = cv2.resize(input_img1, (self.camera2.width,self.camera2.height), interpolation = cv2.INTER_AREA)
            
            self.imgResult = input_img1.copy()
            # Notify output of error and skip iteration
            if frame_time1 == 0:
                self.cvsrc.notifyError(self.sink.getError())
                print("Error on line 368 or 371 with grabbing frame")
                continue
            
            if self.processingForAprilTags:
                self.getAprilTagTargetID()
                try:
                    greys = cv2.cvtColor(input_img1, cv2.COLOR_BGR2GRAY)
                    dets = self.detector.detect(greys)
                except RuntimeError:
                    print("No Apriltag in View")
                    continue
                aprilTagTargets = dict()
            
                for det in dets:
                    if det["margin"] >= self.MIN_MARGIN:
                        rect = det["lb-rb-rt-lt"].astype(int).reshape((-1,1,2))
                        cv2.polylines(self.imgResult, [rect], True, self.RED, 2)
                        ident = str(det["id"])
                        pos = det["center"].astype(int) + (0,0)
                        aprilTagTargets.update({det["id"]:AprilTagTarget(self.currentCamera,pos,det["id"])})
                        cv2.putText(self.imgResult, ident, tuple(pos), self.FONT, 1, self.RED, 2)

                if not aprilTagTargets: 
                    # If no apriltags are detected, targetDetected is set to false
                    
                    self.vision_nt.putNumber('aprilTagTargetDetected',0)
                else:
                    if self.aprilTagTargetID in aprilTagTargets:
                        # Data published for Apriltags
                        # If AprilTags are detected, targetDetected is set to true 
                        self.vision_nt.putNumber('aprilTagTargetDetected',1)
                        # Publishes data to Network Tables
                        self.vision_nt.putNumber('offset',aprilTagTargets[self.aprilTagTargetID].offset)
                        self.vision_nt.putNumber('targetX',aprilTagTargets[self.aprilTagTargetID].normalizedX)
                        # If you want to calculate yaw and distance, make sure to fill out the appropriate variables starting on line 59
                        self.vision_nt.putNumber('robotYaw',aprilTagTargets[self.aprilTagTargetID].yaw)
                        self.vision_nt.putNumber('distanceToTarget',aprilTagTargets[self.aprilTagTargetID].distanceToTarget)
                        NetworkTables.flush()
                    else:
                        self.vision_nt.putNumber('aprilTagTargetDetected',0)
                    

            if self.processingForColor:
                self.getColorDetectConst()
                self.tapeTargetList = []
                self.targets = []
                self.processImgForTape(input_img1)
                # sorts the list of tape targets from left to right
                t2 = time.clock_gettime(time.CLOCK_MONOTONIC) # gets the current "time"
                timeDiff = t2-t1 # difference between the most recent time and the time recorded when the target was last seen
                if self.tapeTargetDetected:
                    self.tapeTargetList.sort(key=lambda target: target.boundingArea) # Sorts the targets smallest to largest.
                    targetID = len(self.tapeTargetList)-1
                    self.tapeTargetList[targetID].drawRectangle()
                    # Data published for Color
                    self.vision_nt.putNumber('offset',self.tapeTargetList[targetID].offset)
                    self.vision_nt.putNumber('ycoor',self.tapeTargetList[targetID].y)
                    self.vision_nt.putNumber('areaRatio',self.tapeTargetList[targetID].areaRatio)
                    self.vision_nt.putNumber('aspectRatio',self.tapeTargetList[targetID].aspectRatio)

                    # If you want to calculate yaw and distance, make sure to fill out the appropriate variables starting on line 59
                    self.vision_nt.putNumber('robotYaw',self.tapeTargetList[targetID].yaw)
                    self.vision_nt.putNumber('distanceToTarget',self.tapeTargetList[targetID].distanceToTarget)
                    
                    t1 = t2
                    self.vision_nt.putNumber('tapeTargetDetected',1)
                    self.vision_nt.putNumber('BoundingArea',self.garea)
                
                else: # only sets updates the targetDetected if a certain amount of time has passed
                    if timeDiff > targetDetTol:
                        self.vision_nt.putNumber('tapeTargetDetected',0)
                self.cvmask.putFrame(self.mask)
            if not self.detectionMode == 0:
                self.cvsrc.putFrame(self.imgResult)

def main():
    visionApp = VisionApplication()
    visionApp.runApplication()

main()   
