#!/usr/bin/env python
# Software License Agreement
__version__ = "0.0.1"
__status__ = "Production"
__license__ = "BSD"
__copyright__ = "Copyright (c) 2015, P.A.N.D.O.R.A. Team. All rights reserved."
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
__author__ = "Kofinas Miltiadis"
__maintainer__ = "Kofinas Miltiadis"
__email__ = "mkofinas@gmail.com"

import os
import shutil
import random

from collections import defaultdict


class RandomDatasetCreator:
    """ This class is used to create a random sub-set of an already existing
        dataset. The original dataset contains class labeled images for a
        two-class classification problem. The sub-set chosen has about the same
        percentage of positive and negative images as the original one.

        Parameters
        ----------
        srcImagePath: The absolute path to the initial dataset, i.e. the path
        to the folder containing the initial dataset images.
        srcAnnotationsPath: The absolute path to the initial annotations file.
        srcAnnotationsFileName: The name of the initial annotations file (along
        with its file extension).
        dstImagePath: The absolute path to the new dataset, i.e. the path to
        the folder that will contain the new dataset images.
        dstAnnotationsPath: The absolute path to the new annotations file.
        dstAnnotationsFileName: The name of the new dataset annotations file
        (along with its file extension).
    """
    def __init__(self, srcImagePath, srcAnnotationsPath,
                 srcAnnotationsFileName, dstImagePath, dstAnnotationsPath,
                 dstAnnotationsFileName):
        """Initialize class member variables."""
        self.annotationsDict = defaultdict(list)
        self.newAnnotationsDict = defaultdict(list)
        self.positiveImageNames = []
        self.negativeImageNames = []
        self.newPositiveImageNames = []
        self.newNegativeImageNames = []
        self.numPositives = 0
        self.numNegatives = 0
        self.positivesPercentage = 0
        self.desiredDatasetSize = 1000

        self.srcImagePath = srcImagePath
        self.srcAnnotationsPath = srcAnnotationsPath
        self.srcAnnotationsFileName = srcAnnotationsFileName
        self.dstImagePath = dstImagePath
        self.dstAnnotationsPath = dstAnnotationsPath
        self.dstAnnotationsFileName = dstAnnotationsFileName

    def readAnnotationsFile(self):
        """ This function reads a file containing the class attributes for each
            image of the original dataset.
        """
        fileExtension = ".txt"
        # Check that the path exists.
        if not os.path.isdir(self.srcAnnotationsPath):
            print "ERROR : Incorrect Path for annotations file."
            exit(1)
        # Check that the file extension is appropriate.
        if (fileExtension not in self.srcAnnotationsFileName):
            print fileName, "is not a proper file."
            exit(2)
        # Read the annotations file.
        annotationsFile = open(os.path.join(self.srcAnnotationsPath,
                                            self.srcAnnotationsFileName), "rU")

        # Ensure that the file was read successfully.
        if (annotationsFile is None):
            print "Error reading the file", self.srcAnnotationsFileName
            exit(3)

        # Read file and save contents in a dictionary.
        for line in annotationsFile:
            (frameName, classAttribute, xTopLeftPoint, yTopLeftPoint,
                xBottomRightPoint, yBottomRightPoint) = line.split(",")
            self.annotationsDict[frameName].append((int(classAttribute),
                                                    int(xTopLeftPoint),
                                                    int(yTopLeftPoint),
                                                    int(xBottomRightPoint),
                                                    int(yBottomRightPoint)))

            if int(classAttribute) == 1:
                self.numPositives += 1
                self.positiveImageNames.append(frameName)
            else:
                self.numNegatives += 1
                self.negativeImageNames.append(frameName)

        totalNumImages = self.numPositives + self.numNegatives
        if totalNumImages < self.desiredDatasetSize:
            print "ERROR: Not enough images to create a dataset"
            print "Available Images:", totalNumImages
            exit(1)
        self.positivesPercentage = (float(self.numPositives) /
                                    totalNumImages)
        annotationsFile.close()

    def createAnnotationsFile(self):
        """ This function creates the annotations file for the sub-set of the
            dataset that was created.
        """
        fileExtension = ".txt"
        # Check that the path exists.
        if not os.path.isdir(self.dstAnnotationsPath):
            print "ERROR : Incorrect Path for annotations file."
            exit(1)
        # Check that the file extension is appropriate.
        if (fileExtension not in self.dstAnnotationsFileName):
            print fileName, "is not a proper file."
            exit(2)
        # Open the annotations file.
        annotationsFile = open(os.path.join(self.dstAnnotationsPath,
                                            self.dstAnnotationsFileName), "w")

        # Save new annotations in file.
        for (frameName, dictValues) in self.newAnnotationsDict.iteritems():
            annotationsFile.write(frameName)
            annotationsFile.write(",")
            annotationsFile.write(str(dictValues[0][0][0]))
            annotationsFile.write(",")
            annotationsFile.write(str(dictValues[0][0][1]))
            annotationsFile.write(",")
            annotationsFile.write(str(dictValues[0][0][2]))
            annotationsFile.write(",")
            annotationsFile.write(str(dictValues[0][0][3]))
            annotationsFile.write(",")
            annotationsFile.write(str(dictValues[0][0][4]))
            annotationsFile.write("\n")
        annotationsFile.close()

    def chooseRandomSubset(self):
        """ This function creates a random sub-set of the original dataset,
            with the same percentage of positive and negative images as the
            original.
        """
        numPositives = int(self.desiredDatasetSize * self.positivesPercentage)
        numNegatives = int(self.desiredDatasetSize - numPositives)

        randomPositivesList = random.sample(
                xrange(len(self.positiveImageNames)), numPositives)
        randomNegativesList = random.sample(
                xrange(len(self.negativeImageNames)), numNegatives)
        # Create random sub-sets for positive and negative images.
        for ii in xrange(numPositives):
            self.newPositiveImageNames.append(
                    self.positiveImageNames[randomPositivesList[ii]])
        for ii in xrange(numNegatives):
            self.newNegativeImageNames.append(
                    self.negativeImageNames[randomNegativesList[ii]])
        # Create the new annotations dictionary.
        for annotatorKey, annotatorValues in self.annotationsDict.iteritems():
            if (annotatorKey in self.newPositiveImageNames or
                    annotatorKey in self.newNegativeImageNames):
                self.newAnnotationsDict[annotatorKey].append(annotatorValues)

    def copyDatasetImages(self):
        """ This function copies the randomly chosen dataset images to a
            specified folder.
        """
        imageTypes = [".png", ".jpg", ".bmp", ".pgm"]
        if not os.path.isdir(self.srcImagePath):
            print "ERROR : Incorrect Path for source image dataset"
            exit(1)
        if not os.path.isdir(self.dstImagePath):
            print "ERROR : Incorrect Path for destination image dataset"
            exit(1)
        for fileName in sorted(os.listdir(self.srcImagePath)):
            isImage = False
            for type in imageTypes:
                if (type in fileName):
                    isImage = True
                    break
            if (not isImage):
                print fileName, "is not an image"
                continue

            if (fileName in self.newPositiveImageNames or
                    fileName in self.newNegativeImageNames):

                shutil.copyfile(os.path.join(self.srcImagePath, fileName),
                                os.path.join(self.dstImagePath, fileName))

    def createDataset(self):
        """ This function creates a random dataset that is a subset of another
            dataset.
        """
        self.readAnnotationsFile()
        self.chooseRandomSubset()
        self.createAnnotationsFile()
        self.copyDatasetImages()


if __name__ == "__main__":
    print "Initialize process"
    print "Type the absolute path to the initial dataset:"
    srcImagePath = raw_input("-->")
    print "Type the absolute path to the initial annotations file:"
    srcAnnotationsPath = raw_input("-->")
    print "Type the name of the initial annotations file:"
    srcAnnotationsFileName = raw_input("-->")
    print "Type the absolute path to the new dataset:"
    dstImagePath = raw_input("-->")
    print "Type the absolute path to the new annotations file:"
    dstAnnotationsPath = raw_input("-->")
    print "Type the name of the new annotations file:"
    dstAnnotationsFileName = raw_input("-->")

    randDataset = RandomDatasetCreator(srcImagePath, srcAnnotationsPath,
                                       srcAnnotationsFileName, dstImagePath,
                                       dstAnnotationsPath,
                                       dstAnnotationsFileName)
    randDataset.createDataset()
