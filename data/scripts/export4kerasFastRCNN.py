import numpy as np
import cv2
import re
import glob
import os


# export a file in the format of : https://github.com/yhenon/keras-frcnn


class exportTLkerasFRCNN(object):
    def __init__(self):
        #self.folderImg = '../just_traffic_light_images/'
        self.folderOutputFile = 'dataset.txt'
        self.folderImg = '../loop_with_traffic_light_images/'

        self.filesImg = glob.glob(self.folderImg + 'fram*.jpg')
        self.roiImg = glob.glob(self.folderImg + 'fram*.jpg.roi.txt')

        self.filesImg.sort()
        self.roiImg.sort()

        self.classesName = {1: "red", 2: "orange", 3: "green"}
        self.classesNameIndex = {1: 0, 2: 0, 3: 0}

        with open(self.folderImg+self.folderOutputFile, 'a') as fileID:
            for i in range(0, len(self.filesImg)):
                f = self.filesImg[i]
                fROI = f + '.roi.txt'
                if fROI in self.roiImg:
                    roi = np.loadtxt(fROI, dtype=np.int16)
                    fFullPath = os.path.abspath(f)
                    classe = roi[4]
                    data = fFullPath + "," + str(roi[0]) + ',' + str(roi[2]) + ',' + str(roi[1]) + ',' + str(
                        roi[3]) + ',' + self.classesName[classe] + '\n'
                    print(data)
                    fileID.write(data)

        fileID.close()


if __name__ == '__main__':
    roiSel = exportTLkerasFRCNN()
