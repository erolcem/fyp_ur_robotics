#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import cv2 as cv

def nothing(x):
	pass
cap = cv.VideoCapture(0)
font = cv.FONT_HERSHEY_SIMPLEX

cv.namedWindow('Tracking')
cv.createTrackbar("LH", "Tracking", 0, 255, nothing)
cv.createTrackbar("LS", "Tracking", 0, 255, nothing)
cv.createTrackbar("LV", "Tracking", 0, 255, nothing)
cv.createTrackbar("UH", "Tracking", 255, 255, nothing)
cv.createTrackbar("US", "Tracking", 255, 255, nothing)
cv.createTrackbar("UV", "Tracking",255, 255, nothing)

while cap.isOpened():
	_, frame = cap.read()
	hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
	l_h = cv.getTrackbarPos("LH","Tracking")
	l_s = cv.getTrackbarPos("LS","Tracking")
	l_v = cv.getTrackbarPos("LV","Tracking")
	u_h = cv.getTrackbarPos("UH","Tracking")
	u_s = cv.getTrackbarPos("US","Tracking")
	u_v = cv.getTrackbarPos("UV","Tracking")
	l_b = np.array([l_h, l_s,l_v])
	u_b = np.array([u_h, u_s,u_v])

	mask = cv.inRange(hsv, l_b, u_b)
	_, thresh = cv.threshold(mask, 20,255,cv.THRESH_BINARY)
	dilated = cv.dilate(thresh,None, iterations=3)
	_, contours, _ = cv.findContours(dilated,cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

	for contour in contours:
		(x,y,w,h) = cv.boundingRect (contour)

		if cv.contourArea(contour) < 10000:
			continue
		# cv.rectangle (frame, (x,y), (x+w,y+h), (0,255,0),2)

		# corner1strXY = str(x) + ',' + str(y)
		# cv.putText(frame, corner1strXY, (x, y), font, 1, (255,255,0), 2)f
		# corner2strXY = str(x+w) + ',' + str(y)
		# cv.putText(frame, corner2strXY, (x+w, y), font, 1, (255,255,0), 2)
		# corner3strXY = str(x) + ',' + str(y+h)
		# cv.putText(frame, corner3strXY, (x, y+h), font, 1, (255,255,0), 2)
		# corner4strXY = str(x+w) + ',' + str(y+h)
		# cv.putText(frame, corner4strXY, (x+w, y+h), font, 1, (255,255,0), 2)
		# n = 10
		# for i in range (n):
		# 	cv.line(frame,(640*i/n,0),(640*i/n,480),(0,255,0),2)

		# for j in range (n):
		# 	cv.line(frame,(0,480*j/n),(640,480*j/n),(0,255,0),2)
			
		



	cv.imshow("feed",frame)
	cv.imshow("mask",mask)
	if cv.waitKey(40) == 27:
		break

cv.destroyAllWindows()
cap.release()
