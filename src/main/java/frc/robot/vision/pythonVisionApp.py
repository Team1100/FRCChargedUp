#!/usr/bin/env python3
from cscore import CameraServer
from networktables import NetworkTablesInstance
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

class AprilTagTarget(object):
    def __init__(self, camera, coor, id):
        self.id = id
        self.normalizedY = (coor[1] - camera.height/2)/(camera.height/2) * -1
        self.normalizedX = (coor[0] - camera.width/2)/(camera.width/2)
        self.pitch = (self.normalizedY/2) * camera.vertFOV
        self.yaw = (self.normalizedX/2) * camera.horizFOV
        #(height of target [feet] - height of camera [feet])/tan(pitch [degrees] + angle of camera [degrees])
        self.distanceToTarget = (camera.elevationOfTarget - camera.elevationOfCamera) / math.tan(math.radians(self.pitch + camera.angleFromHoriz))

class TapeTarget(object):
    def __init__(self, imageResult, approx, tapeTargetDetected, camera):
        self.tapeTargetDetected = tapeTargetDetected
        self.imageResult = imageResult
        if self.tapeTargetDetected:
            self.x, self.y, self.w, self.h, = cv2.boundingRect(approx)
        else:
            self.x, self.y, self.w, self.h, = 1, 1, 1, 1 
        self.normalizedY = (self.y - camera.height/2)/(camera.height/2) * -1
        self.normalizedX = (self.x - camera.width/2)/(camera.width/2)
        self.pitch = (self.normalizedY/2) * camera.vertFOV
        self.yaw = (self.normalizedX/2) * camera.horizFOV
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

        self.usingComputerIP = False 

        # Initialize configuration
        self.config = self.readConfig()
        self.team = self.config["team"]

        self.imgResult = None
        self.mask = None


        self.hueMin = 13
        self.hueMax = 255
        self.satMin = 1
        self.satMax = 66
        self.valMin = 235
        self.valMax = 255
       
        self.myColors = [[self.hueMin,self.satMin,self.valMin,self.hueMax,self.satMax,self.valMax]]

        self.areaRatio = 0 # this is the areaRatio of every contour that is seen by the camera
        self.largestAreaRatio = 0 # this is the areaRatio of the target once it has been isolated
        self.aspectRatio = 0 # this is the aspectRatio of every contour that is seen by the camera (width/height)
        self.largestAspectRatio = 0 # this is the aspectRatio fo the target once it has been isolated

        self.garea = 0
        self.contours = None
        self.target = None
        self.tapeTargetList = [None]

        #TODO: Fill out values below if distance calculation is desired
        #Vertical Field of View (Degrees)
        vertFOV = 1

        #Horizontal Field of View (Degrees)
        horizFOV = 1

        #Height of the target off the ground (feet)
        elevationOfTarget = 1

        #Height of the Camera off the ground (feet)
        elevationOfCamera = 1

        #Angle the camera makes relative to the horizontal (degrees)
        angleFromHoriz = 1

        self.camera = CameraView(self.config['cameras'][0], vertFOV, horizFOV, elevationOfTarget, elevationOfCamera, angleFromHoriz)

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
        camera = cserver.startAutomaticCapture()
        camera.setResolution(self.camera.width,self.camera.height)


        self.cvsrc = cserver.putVideo("visionCam", self.camera.width,self.camera.height)
        self.cvmask = cserver.putVideo("maskCam", self.camera.width, self.camera.height)
        
        self.sink = CameraServer.getInstance().getVideo()

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
        self.hueMin = int(self.vision_nt.getNumber('hueMin',255))
        self.hueMax = int(self.vision_nt.getNumber('hueMax',255))
        self.satMin = int(self.vision_nt.getNumber('satMin',255))
        self.satMax = int(self.vision_nt.getNumber('satMax',255))
        self.valMin = int(self.vision_nt.getNumber('valMin',255))
        self.valMax = int(self.vision_nt.getNumber('valMax',255))
        self.myColors = [[self.hueMin,self.satMin,self.valMin,self.hueMax,self.satMax,self.valMax]]

    def getImageMask(self, img, myColors):
        imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)  
        lower = np.array(myColors[0][0:3])
        upper = np.array(myColors[0][3:6])
        mask = cv2.inRange(imgHSV, lower, upper)
        return mask


    def getContours(self, img):
        contours, hierarchy = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        return contours

    def isolateTarget(self, contours):
        tolerance = .225
        idealAreaRatio = 0.7 # this is the ideal ratio for the area ratio value.
        idealAspectRatio = 1.66/5 # this is the ideal aspect ratio based off of the diagram but can be changed as needed.
        aspectTolerance = .44 # this is the tolerance for finding the target with the right aspect ratio
        # start off with a large tolerance, and if the ideal ratio is correct, lower the tolerance as needed. 
        self.garea = 0
        target = self.target # Default to the "old" target
        
        self.tapeTargetDetected = False
        if len(contours) > 0:
            largest = contours[0]
            area = 0
            for contour in contours:
                contourArea = cv2.contourArea(contour) #area of the particle
                x, y, w, h, = cv2.boundingRect(contour)
                boundingArea = w * h
                if (boundingArea > 2000):
                    continue
                self.areaRatio = contourArea/boundingArea
                self.aspectRatio = w/h
                if self.areaRatio > idealAreaRatio - tolerance and self.areaRatio < idealAreaRatio + tolerance: # if the target is within the right area ratio range, it is possibly the correct target
                    if self.aspectRatio > idealAspectRatio - aspectTolerance and self.aspectRatio < idealAspectRatio + aspectTolerance: # if the target is within the correct aspect ratio range aswell, it is definitely the right target
                        largest = contour
                        area = boundingArea
                        self.largestAreaRatio = self.areaRatio
                        self.largestAspectRatio = self.aspectRatio
                        self.tapeTargetDetected = True
                        self.garea = area
                        target = largest
                        # Draw the contours
                        cv2.drawContours(self.imgResult, target, -1, (255,0,0), 3)
        return target

    def drawBoundingBox(self, target):
        if self.tapeTargetDetected:
            peri = cv2.arcLength(target, True)
            approx = cv2.approxPolyDP(target, 0.02 * peri, True)
        else:
            approx = None
        tapeTarget = TapeTarget(self.imgResult, approx, self.tapeTargetDetected, self.camera)
        self.tapeTargetList.append(tapeTarget)
        tapeTarget.drawRectangle()

    def processImgForTape(self, input_img):
        self.getMaskingValues()
        self.mask = self.getImageMask(input_img,self.myColors)
        self.contours = self.getContours(self.mask)
        self.target = self.isolateTarget(self.contours)
        self.drawBoundingBox(self.target)

    def runApplication(self):
        input_img1 = np.zeros(shape=(self.camera.height,self.camera.width,3),dtype=np.uint8)
        targetDetTol = 1.0 
        t1 = 0
        t2 = 0
        while True:
            camCenter = (self.camera.width)/2
            
            frame_time1, input_img1 = self.sink.grabFrame(input_img1)
            input_img1 = cv2.resize(input_img1, (self.camera.width,self.camera.height), interpolation = cv2.INTER_AREA)
            
            self.imgResult = input_img1.copy()
            # Notify output of error and skip iteration
            if frame_time1 == 0:
                self.cvsrc.notifyError(self.sink.getError())
                print("Error on line 135 with grabbing frame")
                continue

            greys = cv2.cvtColor(input_img1, cv2.COLOR_BGR2GRAY)
            dets = self.detector.detect(greys)

            # array of AprilTagTarget classes
            aprilTagTargets = dict()

            for det in dets:
                if det["margin"] >= self.MIN_MARGIN:
                    rect = det["lb-rb-rt-lt"].astype(int).reshape((-1,1,2))
                    cv2.polylines(self.imgResult, [rect], True, self.RED, 2)
                    ident = str(det["id"])
                    pos = det["center"].astype(int) + (-10,10)
                    aprilTagTargets.update({det["id"]:AprilTagTarget(self.camera,pos,det["id"])})
                    cv2.putText(self.imgResult, ident, tuple(pos), self.FONT, 1, self.RED, 2)
            
            if not aprilTagTargets: 
                # If no apriltags are detected, targetDetected is set to false
                self.vision_nt.putNumber('aprilTagTargetDetected',0)
            else: 
                # If AprilTags are detected, targetDetected is set to true 
                self.vision_nt.putNumber('aprilTagTargetDetected',1)

                # Publishes data to Network Tables
                self.vision_nt.putNumber('targetX',aprilTagTargets[1].normalizedX)
                self.vision_nt.putNumber('targetY',aprilTagTargets[1].normalizedY)
                # If you want to calculate distance, make sure to fill out the appropriate variables starting on line 59
                #self.vision_nt.putNumber('distanceToTarget',targets[0].distanceToTarget)


            self.processImgForTape(input_img1)

            # sorts the list of tape targets from left to right
            t2 = time.clock_gettime(time.CLOCK_MONOTONIC) # gets the current "time"
            timeDiff = t2-t1 # difference between the most recent time and the time recorded when the target was last seen
            if self.tapeTargetDetected:
                t1 = t2
                self.vision_nt.putNumber('tapeTargetDetected',1)
                #self.tapeTargetList.sort(key=lambda x: x.normalizedX)
                self.vision_nt.putNumber('BoundingArea',self.garea)
                self.vision_nt.putNumber('Area Ratio',self.largestAreaRatio)
                self.vision_nt.putNumber('Aspect Ratio',self.largestAspectRatio)
            else: # only sets updates the targetDetected if a certain amount of time has passed
                if timeDiff > targetDetTol:
                    self.vision_nt.putNumber('tapeTargetDetected',0)

            self.cvsrc.putFrame(self.imgResult)
            self.cvmask.putFrame(self.mask)

def main():
    visionApp = VisionApplication()
    visionApp.runApplication()

main()   
