import imp
from pydoc import classname
import cv2
import numpy as np
import os
import sys


path = 'imgs_train'
# Initiate ORB detector
orb = cv2.ORB_create(nfeatures=2000)

#### IMport Images
images = []
classNames = []
myList = os.listdir(path)
print(myList)
print('Total Classes Detected',len(myList))

for cl in myList:
    imgCur = cv2.imread(f'{path}/{cl}',0)
    images.append(imgCur)
    classNames.append(os.path.splitext(cl)[0])

print(classNames)

def findDes(images):
    desList = []
    for img in images:
        kp = orb.detect(img,None)
        kp, des = orb.compute(img,kp)
        desList.append(des)
    return desList

sensitivity = 5
scarlett_flag = 0
peacock_flag = 0
mustard_flag = 0
plum_flag = 0


def findID(img1, img, deList):
    kp2 = orb.detect(img,None)
    kp2, des2 = orb.compute(img,kp2)
    bf = cv2.BFMatcher()
    matchList = []
    finalVal = -1
    try:
        for des in desList:
            matches = bf.knnMatch(des,des2,k=2)
            good = []
            for m,n in matches:
                if m.distance < 0.75*n.distance:
                    good.append([m])

            matchList.append(len(good))
    except:
        pass
    # print(matchList)

    hsv_scarlett_lower = np.array([5-sensitivity,100,100])
    hsv_scarlett_upper = np.array([5+sensitivity,255,255])
    hsv_peacock_lower = np.array([105-sensitivity,100,100])
    hsv_peacock_upper = np.array([105+sensitivity,255,255]) # can't see her dress
    hsv_mustard_lower = np.array([25-sensitivity,100,100])
    hsv_mustard_upper = np.array([25+sensitivity,255,255])
    hsv_plum_lower = np.array([153-sensitivity,100,10])
    hsv_plum_upper = np.array([153+sensitivity,225,255]) # not as clar as others
    # Filter out everything but particular colours using the cv2.inRange() method
    mask1 = cv2.inRange(img1,hsv_scarlett_lower,hsv_scarlett_upper)
    mask2 = cv2.inRange(img1,hsv_peacock_lower,hsv_peacock_upper)
    mask3 = cv2.inRange(img1,hsv_mustard_lower,hsv_mustard_upper)
    mask4 = cv2.inRange(img1,hsv_plum_lower,hsv_plum_upper)

    # detecting plum
    img_plum = cv2.bitwise_and(img1,img1,mask=mask4)
    contours_plum, hierarchy = cv2.findContours(mask4, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours_plum) > 0:
        c = max(contours_plum,key=cv2.contourArea)
        if cv2.contourArea(c) > 5500:
            x, y, w, h = cv2.boundingRect(c)
            cv2.rectangle(img_plum,(x,y),(x+w, y+h),(0,0,255),2)
            global plum_flag
            plum_flag = 1

    # detecting peacock
    img_peacock = cv2.bitwise_and(img1,img1,mask=mask2)
    contours_peacock, hierarchy = cv2.findContours(mask2, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours_peacock) > 0:
        c = max(contours_peacock,key=cv2.contourArea)
        if cv2.contourArea(c) > 5500:
            x, y, w, h = cv2.boundingRect(c)
            cv2.rectangle(img_peacock,(x,y),(x+w, y+h),(0,0,255),2)
            global peacock_flag
            peacock_flag = 1

    # detecting scarlett
    img_scarlett= cv2.bitwise_and(img1,img1,mask=mask1)
    contours_scarlett, hierarchy = cv2.findContours(mask1, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours_scarlett) > 0:
        c = max(contours_scarlett,key=cv2.contourArea)
        if cv2.contourArea(c) > 5500:
            x, y, w, h = cv2.boundingRect(c)
            cv2.rectangle(img_scarlett,(x,y),(x+w, y+h),(0,0,255),2)
            global scarlett_flag
            scarlett_flag = 1

    # detecting mustard
    img_mustard = cv2.bitwise_and(img1,img1,mask=mask3)
    contours_mustard, hierarchy = cv2.findContours(mask3, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours_mustard) > 0:
        c = max(contours_mustard,key=cv2.contourArea)
        if cv2.contourArea(c) > 5500:
            x, y, w, h = cv2.boundingRect(c)
            cv2.rectangle(img_mustard,(x,y),(x+w, y+h),(0,0,255),2)
            global mustard_flag
            mustard_flag = 1


    if len(matchList)!=0:
        if (max(matchList) > 30) and (plum_flag==1 or peacock_flag==1 or scarlett_flag==1 or mustard_flag==1):
            finalVal = matchList.index(max(matchList))
    return finalVal

desList = findDes(images)
print(desList)

cap = cv2.VideoCapture(0)
# cap = cv2.imread('imgs/plum.png',0)

while True:
    success, img2 = cap.read()
    # success, img2 = cv2.imread('imgs/plum.png',0)
    imgOrg = img2.copy()
    # Convert the bgr image into a hsv image
    img1 = cv2.cvtColor(img2,cv2.COLOR_BGR2HSV)
    img2 = cv2.cvtColor(img2,cv2.COLOR_BGR2GRAY)
    
    id = findID(img1,img2,desList)
    font=cv2.FONT_HERSHEY_COMPLEX
    fontScale=1
    fontColor=(255,255,255)
    lineType=cv2.LINE_AA
    org=(50,50)
    if id != -1:
        cv2.putText(imgOrg,classNames[id],org,font,fontScale,fontColor,2)
    cv2.imshow('img2',imgOrg)
    cv2.waitKey(1)