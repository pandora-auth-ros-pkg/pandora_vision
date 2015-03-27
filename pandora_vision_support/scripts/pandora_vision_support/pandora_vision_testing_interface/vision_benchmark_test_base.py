# Software License Agreement
__version__ = "0.0.1"
__status__ = "Production"
__license__ = "BSD"
__copyright__ = "Copyright (c) 2014, P.A.N.D.O.R.A. Team. All rights reserved."
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of P.A.N.D.O.R.A. Team nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
__author__ = "Houtas Vasileios, Tsirigotis Christos, Kofinas Miltiadis"
__maintainer__ = "Kofinas Miltiadis"
__email__ = "mkofinas@gmail.com"

import os
import unittest

import rospy

import math
import numpy
import time
from collections import defaultdict
import cv2
from cv_bridge import CvBridge, CvBridgeError

from pandora_testing_tools.testing_interface import test_base
from sensor_msgs.msg import Image

class VisionBenchmarkTestBase(test_base.TestBase):
    def readImages(self, imagePath):
        self.images = []
        self.names = []
        imageTypes = [".png", ".jpg", ".bmp", ".pgm"]
        bridge = CvBridge()
        if not os.path.isdir(imagePath):
            print "ERROR : Incorrect Path for image dataset"
            exit(1)

        for fileName in sorted(os.listdir(imagePath)):
            isImage = False
            for type in imageTypes:
                if (type in fileName):
                    isImage = True
                    break
            if (not isImage):
                print fileName, "is not an image"
                continue
            # Read the next image.
            currentImg = cv2.imread(os.path.join(imagePath,
                                    fileName), -1)

            # If the image was not read succesfully continue to the
            # next file.
            if (currentImg is None):
                print "Error reading image", fileName
                print "The process will read the next image!"
                continue

            # If the tested algorithm is not Victim, there is no need to use
            # depth and thermal images.
            if (("depth" in fileName or "thermal" in fileName) and
                (self.algorithm != "Victim" and self.algorithm != "Hole")):

                continue
            # Store the image.
            self.images.append(bridge.cv2_to_imgmsg(currentImg, "bgr8"))
            self.names.append(fileName)

    def readBenchmarkFile(self, filePath):
        filePath, pathTail = os.path.split(filePath)
        benchmarkFileName = filePath + "/vision_benchmarking.txt"
        if not os.path.isdir(filePath):
            print "ERROR : Incorrect Path for benchmark file."
            exit(3)
        benchmarkFile = open(benchmarkFileName, "rU")
        self.benchmarkDict = defaultdict(list)
        for line in benchmarkFile:
            if line.startswith("#"):
                continue
            (frameName, objectType, distance, hAngle, vAngle, specs) = line.split(", ")
            if self.algorithm != "Victim" and "depth" in frameName:
                continue
            self.benchmarkDict[objectType, float(distance), float(hAngle),
                               float(vAngle), specs].append(frameName)
        benchmarkFile.close()

    def readAnnotatorFile(self, filePath):
        filePath, pathTail = os.path.split(filePath)
        annotatorFileName = filePath + "/vision_annotations.txt"
        if not os.path.isdir(filePath):
            print "ERROR : Incorrect Path for annotator file."
            exit(2)
        annotatorFile = open(annotatorFileName, "rU")
        self.annotatorDict = defaultdict(list)
        for line in annotatorFile:
            if line.startswith("#"):
                continue
            (frameName, objectType, xTopLeftPoint, yTopLeftPoint,
                xBottomRightPoint, yBottomRightPoint) = line.split(", ")
            if self.algorithm != "Victim" and "depth" in frameName:
                continue
            self.annotatorDict[frameName].append((objectType,
                                            int(xTopLeftPoint),
                                            int(yTopLeftPoint),
                                            int(xBottomRightPoint),
                                            int(yBottomRightPoint)))
        annotatorFile.close()

    def calculateAlertDistanceError(self, imageName, imageX, imageY):
        minSqrError = float("Inf")
        for annotatorKey, annotatorValues in self.annotatorDict.iteritems():
            if annotatorKey == imageName:
                minSqrError = float("Inf")
                for ii in xrange(len(annotatorValues)):
                    if self.algorithm == annotatorValues[ii][0]:
                        alertCenterX = ((annotatorValues[ii][1] +
                                        annotatorValues[ii][3]) / 2.0)
                        alertCenterY = ((annotatorValues[ii][2] +
                                        annotatorValues[ii][4]) / 2.0)
                        alertSqrDist = ((imageX - alertCenterX) ** 2 +
                                        (imageY - alertCenterY) ** 2)
                        if minSqrError > alertSqrDist:
                            minSqrError = alertSqrDist
                rospy.logdebug("Minimum Square Error: %f", minSqrError)
        return minSqrError

    def findAnnotations(self, imageName):
        annotations = 0
        for annotatorKey, annotatorValues in self.annotatorDict.iteritems():
            if annotatorKey == imageName:
                for ii in xrange(len(annotatorValues)):
                    if self.algorithm == annotatorValues[ii][0]:
                        annotations += 1
        return annotations

    def updateActualPositiveDictionary(self):
        for dictKey, dictValues in self.benchmarkDict.iteritems():
            if dictKey[0] == self.algorithm:
                distance = dictKey[1]
                hAngle = dictKey[2]
                vAngle = dictKey[3]
                specs = dictKey[4]
                num = 0
                for ii in xrange(len(dictValues)):
                    if dictValues[ii] in self.names:
                        num += 1
                if num > 0:
                    self.actualPositivesBenchmarkDict[
                        self.algorithm, float(distance), float(hAngle),
                        float(vAngle), specs] += num

    def updateTruePositiveDictionary(self, imageName):
        for dictKey, dictValues in self.benchmarkDict.iteritems():
            if (imageName in dictValues and dictKey[0] == self.algorithm):
                distance = dictKey[1]
                hAngle = dictKey[2]
                vAngle = dictKey[3]
                specs = dictKey[4]
                self.truePositivesBenchmarkDict[self.algorithm,
                                                float(distance),
                                                float(hAngle),
                                                float(vAngle),
                                                specs] += 1

    def calculateBenchmarkResults(self):
        rospy.loginfo("Benchmarking Test Results for %s node.", self.algorithm)
        rospy.loginfo("Number of images used in algorithms: %d", 
                      len(self.images))
        rospy.loginfo("Number of True Positives: %d", self.truePositives)
        rospy.loginfo("Number of False Positives: %d", self.falsePositives)
        rospy.loginfo("Number of True Negatives: %d", self.trueNegatives)
        rospy.loginfo("Number of False Negatives: %d", self.falseNegatives)

        if len(self.elapsedTimeList) > 0:
            minElapsedTime = min(self.elapsedTimeList)
            maxElapsedTime = max(self.elapsedTimeList)
            meanElapsedTime = numpy.mean(self.elapsedTimeList)
            stdElapsedTime = numpy.std(self.elapsedTimeList)

            rospy.loginfo("Minimum Elapsed Time: %f seconds", minElapsedTime)
            rospy.loginfo("Maximum Elapsed Time: %f seconds", maxElapsedTime)
            rospy.loginfo("Mean Elapsed Time : %f seconds", meanElapsedTime)
            rospy.loginfo("Standard Deviation of Elapsed Time: %f seconds", 
                          stdElapsedTime)
        else:
            rospy.loginfo("No alerts found. Cannot display elapsed time.")
        
        precision = 0.0
        if self.truePositives + self.falsePositives > 0:
            precision = (float(self.truePositives) / 
                         (self.truePositives + self.falsePositives))
        
        recall = 0.0
        if self.truePositives + self.falseNegatives > 0:
            recall = (float(self.truePositives) / 
                      (self.truePositives + self.falseNegatives))
        
        fMeasure = 0.0
        if self.truePositives + self.falseNegatives + self.falsePositives > 0:
            fMeasure = (2.0 * self.truePositives / 
                        (2.0 * self.truePositives + self.falseNegatives + 
                        self.falsePositives))
        
        if self.truePositives > 0:
            self.meanSquaredError /= float(self.truePositives)
        
        rospy.loginfo("Precision: %f", precision)
        rospy.loginfo("Recall (TPR): %f", recall)
        rospy.loginfo("F-measure: %f", fMeasure)
        rospy.loginfo("Mean Squared Error from POI: %f", self.meanSquaredError)

    def calculateRecallResults(self):
        for dictKeyTP, dictValuesTP in self.actualPositivesBenchmarkDict.iteritems():
            if dictKeyTP in self.truePositivesBenchmarkDict:
                truePos = self.truePositivesBenchmarkDict[dictKeyTP]
            else:
                truePos = 0
            rospy.loginfo("%s: %d/%d", dictKeyTP, truePos, dictValuesTP)

    def benchmarkTest(self, imagePath, inputTopic, outputTopic):
        # Read a Set of Images
        rospy.loginfo("Reading Images")
        self.readImages(imagePath)
        # Read Annotations for a Set of Images
        rospy.loginfo("Reading Annotator File")
        self.readAnnotatorFile(imagePath)
        # Read Benchmark Parameters for a Set of Images
        rospy.loginfo("Reading Benchmark File")
        self.readBenchmarkFile(imagePath)
        # Create Helper Structures for Benchmarking
        self.truePositivesBenchmarkDict = defaultdict(int)
        self.actualPositivesBenchmarkDict = defaultdict(int)
        self.updateActualPositiveDictionary()
        # Test Parameters and Variables
        self.truePositives = 0
        self.falsePositives = 0
        self.trueNegatives = 0
        self.falseNegatives = 0
        self.meanSquaredError = 0.0
        self.elapsedTimeList = []
        
        imageWidth = 640
        imageHeight = 480
        imageHFOV = 58 * math.pi / 180
        imageVFOV = 45 * math.pi / 180
        errorThreshold = 1200.0
        maxWaitTime = 2.0
        bridge = CvBridge()
        # Publish image files sequentially and wait for an alert. Confirm the 
        # authenticity of the alert using the annotator groundtruth set.
        for image, imageName in zip(self.images, self.names):
            rospy.logdebug("Sending Image %s", imageName)
            timeFlag = False
            startTime = time.time()
            self.mockPublish(inputTopic, outputTopic, image)
            
            while not timeFlag:
                elapsedTime = time.time() - startTime
                if elapsedTime > maxWaitTime or ((self.repliedList[outputTopic]) and
                   (len(self.messageList[outputTopic]) >= 1)):
                    
                    timeFlag = True
            
            if ((self.repliedList[outputTopic]) and
               (len(self.messageList[outputTopic]) >= 1)):
                rospy.logdebug("Alert found in Image %s", imageName)
                rospy.logdebug("Time passed: %f seconds", elapsedTime)

                truePositivesInImage = 0
                # Estimate alert center point from message parameters
                alerts = getattr(self.messageList[outputTopic][0], 
                                 self.algorithm.lower()+"Alerts")
                for iiAlert in xrange(len(alerts)):
                    imageYaw = float(alerts[iiAlert].yaw)
                    imagePitch = float(alerts[iiAlert].pitch)
                    imageX = ((imageWidth * math.tan(imageYaw) / 
                              (math.tan(imageHFOV / 2.0) * 2.0)))
                    imageY = ((imageHeight * math.tan(imagePitch) / 
                              (math.tan(imageVFOV / 2.0) * 2.0)))
                    imageX += (imageWidth / 2)
                    imageY = -imageY + (imageHeight / 2)
                    print "yaw: ", imageYaw * 180 / math.pi, "Degrees"
                    print "pitch: ", imagePitch * 180 / math.pi, "Degrees"
                    print "x, y: ", imageX, imageY

                    minSqrError = self.calculateAlertDistanceError(imageName, 
                                                                   imageX, 
                                                                   imageY)
                    if minSqrError < errorThreshold:
                        truePositivesInImage += 1
                        self.truePositives += 1
                        self.meanSquaredError += minSqrError
                        self.updateTruePositiveDictionary(imageName)
                        self.elapsedTimeList.append(elapsedTime)
                    else:
                        self.falsePositives += 1

                annotationsInImage = self.findAnnotations(imageName)
                if annotationsInImage > truePositivesInImage:
                    self.falseNegatives += (annotationsInImage - 
                                            truePositivesInImage)
            else:
                numberOfFalseNegatives = self.findAnnotations(imageName)
                if numberOfFalseNegatives > 0:
                    self.falseNegatives += numberOfFalseNegatives
                else:
                    self.trueNegatives += 1
        
        # Calculate the Benchmarking Results.
        self.calculateBenchmarkResults()
        # Estimate Recall values for each set of (Distance, Horizontal Angle, 
        # Vertical Angle)
        self.calculateRecallResults()

  
