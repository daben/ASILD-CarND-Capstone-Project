import numpy as np
import cv2
import re
import glob


class roiSelect(object):
    def __init__(self):
        #self.folderImg = '../just_traffic_light_images/'
        self.folderImg = '../loop_with_traffic_light_images/'

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

        print(str(len(self.filesImg)) + " images, ever ",
              str(len(self.filesImg) - self.nbRoiToSelect()) + " ROI to select. " + str(
                  len(self.roiImg)) + " ROI file found.")

        for i in range(0, len(self.filesImg)):
            print("image:{}/{}".format(i, len(self.filesImg)))
            f = self.filesImg[self.fileIndex[i]]
            # if roi file doesn't exist
            if (self.roiImg.count(f + '.roi.txt') == 0):
                self.processImg(f)

                k = cv2.waitKey() & 0xFF
                if k == 27:
                    break
                else:
                    # save the ROI file
                    self.saveRoi(f, i)

    def saveRoi(self, f, index):
        # Xmin,Xmax,Ymin,Ymax,class
        roi = np.zeros(5, dtype=np.uint16)
        roi[0] = min(self.click1['x'], self.click2['x'])
        roi[1] = max(self.click1['x'], self.click2['x'])
        roi[2] = min(self.click1['y'], self.click2['y'])
        roi[3] = max(self.click1['y'], self.click2['y'])
        roi[4] = self.classes[self.fileIndex[index]]
        np.savetxt(f + ".roi.txt", roi, fmt='%d')
        print(f, ' :', roi)

    def nbRoiToSelect(self):
        nb = 0
        for f in self.filesImg:
            roiFile = f + '.roi.txt'
            if (self.roiImg.count(roiFile) != 0):
                nb = nb + 1
        return nb

    def processImg(self, f):

        self.click1 = {}
        self.click2 = {}
        print("load file:" + f)
        img = cv2.imread(f)
        cv2.imshow('Roi select', img)

        cv2.setMouseCallback('Roi select', self.cbMouse)

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

    def cbMouse(self, event, x, y, flags, param):
        if (event == cv2.EVENT_LBUTTONUP):
            if (len(self.click1) == 0):
                self.click1['x'] = x
                self.click1['y'] = y
            else:
                self.click2['x'] = x
                self.click2['y'] = y

            print(self.click1, self.click2)


if __name__ == '__main__':
    roiSel = roiSelect()
