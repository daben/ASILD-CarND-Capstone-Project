import numpy as np
import cv2
import re
import glob


class correctClasses(object):
    def __init__(self):
        self.folderImg = '../just_traffic_light_images/'
        #self.folderImg = '../loop_with_traffic_light_images/'

        self.filesImg = glob.glob(self.folderImg + 'fram*.jpg')
        self.roiImg = glob.glob(self.folderImg + 'fram*.jpg.roi.txt')
        self.fileClasses = self.folderImg + 'classes.txt'

        self.filesImg.sort()
        self.roiImg.sort()

        # classes.txt file, 3 lines: red, orange, green image file index intervals
        self.classesIntervals = np.loadtxt(self.fileClasses).astype(np.int16)
        print(self.classesIntervals)
        self.classes = np.zeros(len(self.filesImg), dtype=np.int16)


        # classes identification
        for i in range(0, len(self.filesImg)):
            f = self.filesImg[i]
            ff = f.split('/')
            file = ff[-1]
            idxStr = re.findall('\d+', file)
            idx = int(idxStr[-1])
            # look
            self.classes[i] = self.eachClasses(idx)
            if (self.classes[i] == -1):
                print("Unlabeled file:" + self.filesImg[i] + " index:" + str(idx))



        self.fileIndex = np.random.choice(len(self.filesImg), len(self.filesImg), replace='False')

        for i in range(0, len(self.filesImg)):
            f = self.filesImg[i]
            fROI = f+'.roi.txt'
            if fROI in self.roiImg:
                print("process :",fROI)
                roi = np.loadtxt(fROI,dtype=np.int16)
                roi[4]=self.classes[i]
                np.savetxt(fROI, roi, fmt='%d')


    def nbRoiToSelect(self):
        nb = 0
        for f in self.filesImg:
            roiFile = f + '.roi.txt'
            if (self.roiImg.count(roiFile) != 0):
                nb = nb + 1
        return nb


    def eachClasses(self, idx):
        for i in range(0, self.classesIntervals.shape[1], 2):
            # red
            if (self.classesIntervals[0, i] != -1 and self.classesIntervals[0, i] != -1):
                if idx >= self.classesIntervals[0, i] and idx <= self.classesIntervals[0, i + 1]:
                    return 1
            # orange
            if (self.classesIntervals[1, i] != -1 and self.classesIntervals[1, i] != -1):
                if idx >= self.classesIntervals[1, i] and idx <= self.classesIntervals[1, i + 1]:
                    return 2
            # green
            if (self.classesIntervals[2, i] != -1 and self.classesIntervals[2, i] != -1):
                if idx >= self.classesIntervals[2, i] and idx <= self.classesIntervals[2, i + 1]:
                    return 3
        return -1



if __name__ == '__main__':
    roiSel = correctClasses()
