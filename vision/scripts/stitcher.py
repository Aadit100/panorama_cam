#!/usr/bin/env python3
import numpy as np
import argparse
import imutils
import cv2
import time
import matplotlib.pyplot as plt
image_paths=['img1.jpg','img2.jpg','img3.jpg','img4.jpg','img5.jpg','img6.jpg','img7.jpg','img8.jpg','img9.jpg','img10.jpg','img11.jpg','img12.jpg','img13.jpg','img14.jpg','img15.jpg','img16.jpg','img17.jpg','img18.jpg']
ap = argparse.ArgumentParser()
ap.add_argument("-c", "--crop", type=int, default=0,
	help="whether to crop out largest rectangular region")
args = vars(ap.parse_args())
# initialized a list of images
imgs = []

  
for i in range(len(image_paths)):
    imgs.append(cv2.imread(image_paths[i]))
    imgs[i]=cv2.resize(imgs[i],(0,0),fx=0.6,fy=0.6)
    for a in range(len(imgs[i])):
        for b in range(len(imgs[i][a])):
            if imgs[i][a][b][0]<10 and imgs[i][a][b][1]<10 and imgs[i][a][b][2]<10:
                imgs[i][a][b][0]=20
                imgs[i][a][b][1]=20
                imgs[i][a][b][2]=20	
    # this is optional if your input images isn't too large
    # you don't need to scale down the image
    # in my case the input images are of dimensions 3000x1200
    # and due to this the resultant image won't fit the screen
    # scaling down the images 
# showing the original pictures
##cv2.imshow('1',imgs[0])
##cv2.imshow('2',imgs[1])
##cv2.imshow('3',imgs[2])
##cv2.imshow('4',imgs[3])
##cv2.imshow('5',imgs[4])
##cv2.imshow('6',imgs[5])
##cv2.imshow('7',imgs[6])
##cv2.imshow('8',imgs[7])
##cv2.imshow('9',imgs[8])
##cv2.imshow('10',imgs[9])
##cv2.imshow('11',imgs[10])
stitchy=cv2.Stitcher.create()
(dummy,output)=stitchy.stitch(imgs)
  
if dummy != cv2.STITCHER_OK:
  # checking if the stitching procedure is successful
  # .stitch() function returns a true value if stitching is 
  # done successfully
    print("stitching ain't successful")
else: 
    print('Your Panorama is ready!!!')
cv2.imwrite('stitched.jpg', output)
if args["crop"] > 0:
		# create a 10 pixel border surrounding the stitched image
		print("[INFO] cropping...")
		stitched = cv2.copyMakeBorder(output, 10, 10, 10, 10,
			cv2.BORDER_CONSTANT, (0, 0, 0))
		# convert the stitched image to grayscale and threshold it
		# such that all pixels greater than zero are set to 255
		# (foreground) while all others remain 0 (background)
		gray = cv2.cvtColor(stitched, cv2.COLOR_BGR2GRAY)
		thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY)[1]
        		# find all external contours in the threshold image then find
		# the largest contour which will be the contour/outline of
		# the stitched image
		cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
			cv2.CHAIN_APPROX_SIMPLE)
		cnts = imutils.grab_contours(cnts)
		c = max(cnts, key=cv2.contourArea)
		# allocate memory for the mask which will contain the
		# rectangular bounding box of the stitched image region
		mask = np.zeros(thresh.shape, dtype="uint8")
		(x, y, w, h) = cv2.boundingRect(c)
		cv2.rectangle(mask, (x, y), (x + w, y + h), 255, -1)
        		# create two copies of the mask: one to serve as our actual
		# minimum rectangular region and another to serve as a counter
		# for how many pixels need to be removed to form the minimum
		# rectangular region
		minRect = mask.copy()
		sub = mask.copy()
		# keep looping until there are no non-zero pixels left in the
		# subtracted image
		while cv2.countNonZero(sub) > 0:
			# erode the minimum rectangular mask and then subtract
			# the thresholded image from the minimum rectangular mask
			# so we can count if there are any non-zero pixels left
			minRect = cv2.erode(minRect, None)
			sub = cv2.subtract(minRect, thresh)

        		# find contours in the minimum rectangular mask and then
		# extract the bounding box (x, y)-coordinates
		cnts = cv2.findContours(minRect.copy(), cv2.RETR_EXTERNAL,
			cv2.CHAIN_APPROX_SIMPLE)
		cnts = imutils.grab_contours(cnts)
		c = max(cnts, key=cv2.contourArea)
		(x, y, w, h) = cv2.boundingRect(c)
		# use the bounding box coordinates to extract the our final
		# stitched image
		stitched = stitched[y:y + h, x:x + w]

# final output
cv2.imshow('final result',stitched)
cv2.waitKey(20000)