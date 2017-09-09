import numpy as np
import cv2
import re
import glob


class exportTLimg(object):
    def __init__(self):
        #self.folderImg = '../just_traffic_light_images/'
        self.folderOut = 'out/'
        self.folderImg = '../loop_with_traffic_light_images/'

        self.filesImg = glob.glob(self.folderImg + 'fram*.jpg')
        self.roiImg = glob.glob(self.folderImg + 'fram*.jpg.roi.txt')


        self.filesImg.sort()
        self.roiImg.sort()

        self.classesName={1:"red",2:"orange",3:"green"}
        self.classesNameIndex = {1: 0, 2:0, 3:0}


        for i in range(0, len(self.filesImg)):
            f = self.filesImg[i]
            fROI = f+'.roi.txt'
            if fROI in self.roiImg:
                roi = np.loadtxt(fROI,dtype=np.int16)
                img = cv2.imread(f)
                classe=roi[4]
                roiSquared = roi.copy()
                centerX = np.mean(roi[0:2])
                centerY = np.mean(roi[2:4])
                size = max(roi[1]-roi[0],roi[3]-roi[2])
                roiSquared[0] = centerX-size/2;
                roiSquared[1] = centerX + 1 + size / 2;
                roiSquared[2] = centerY - size / 2;
                roiSquared[3] = centerY + 1 + size / 2;
                if(roiSquared[0]>0 and roiSquared[2]>0 and roiSquared[1]<img.shape[1] and roiSquared[3]<img.shape[0]):
                    imgROI = img[roiSquared[2]:roiSquared[3], roiSquared[0]:roiSquared[1], :]
                    fOut= self.folderImg+self.folderOut+self.classesName[classe]+"{:04d}".format(self.classesNameIndex[classe])+".jpg"
                    self.classesNameIndex[classe] += 1
                    print("write:" + fOut)
                    resized_image = cv2.resize(imgROI, (64, 64))
                    cv2.imwrite(fOut,resized_image)


if __name__ == '__main__':
    roiSel = exportTLimg()
