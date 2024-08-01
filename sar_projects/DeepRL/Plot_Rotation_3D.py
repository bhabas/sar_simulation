import cv2 as cv
import numpy as np
import os

# Load the image
BASEPATH = '/home/bhabas/GoogleDrive/Grad_Research/Papers/ICRA_25/Figures/2D_3D_Model_Comparison/A1_0deg/3D'
fileName = 'Landing_Rate_Fig_PlaneAngle_0_NoText.png'
filePath = os.path.join(BASEPATH,fileName)
img = cv.imread(filePath)
# cv.imshow('img',img)

scale_percent = 100
width = int(img.shape[1] * scale_percent / 100)
height = int(img.shape[0] * scale_percent / 100)
dim = (width, height)
img = cv.resize(img, dim, interpolation = cv.INTER_CUBIC)

proj2dto3d = np.array([[1,0,-img.shape[1]//2],
                        [0,1,-img.shape[0]//2],
                        [0,0,0],
                        [0,0,1]])

focal_length = 1500

rx = np.array([[1,0,0,0],
               [0,1,0,0],
               [0,0,1,0],
               [0,0,0,1]],np.float32)

ry = np.array([[1,0,0,0],
               [0,1,0,0],
               [0,0,1,0],
               [0,0,0,1]],np.float32)

rz = np.array([[1,0,0,0],
               [0,1,0,0],
               [0,0,1,0],
               [0,0,0,1]],np.float32)

trans = np.array([[1,0,0,0],
                  [0,1,0,0],
                  [0,0,1,2*focal_length],
                  [0,0,0,1]],np.float32)

proj3dto2d = np.array([[focal_length,0,img.shape[1]//2,0],
                        [0,focal_length,img.shape[0]//2,0],
                        [0,0,1,0]],np.float32)

x = 15
y = 40
z = 0


ax = float(x * np.pi / 180)
ay = float(y * np.pi / 180)
az = float(z * np.pi / 180)

rx[1,1] = np.cos(ax)
rx[1,2] = -np.sin(ax)
rx[2,1] = np.sin(ax)
rx[2,2] = np.cos(ax)

ry[0,0] = np.cos(ay)
ry[0,2] = -np.sin(ay)
ry[2,0] = np.sin(ay)
ry[2,2] = np.cos(ay)

rz[0,0] = np.cos(az)
rz[0,1] = -np.sin(az)
rz[1,0] = np.sin(az)
rz[1,1] = np.cos(az)



r = rx.dot(ry).dot(rz)
final = proj3dto2d.dot(trans.dot(r.dot(proj2dto3d)))
img = cv.warpPerspective(img,final,(img.shape[1],img.shape[0]),None,cv.INTER_LINEAR,cv.BORDER_CONSTANT,(255,255,255))

gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
# cv.imshow('Gray',gray)

_, thresh = cv.threshold(gray,254, 255, cv.THRESH_BINARY_INV)
# cv.imshow('Thresh',thresh)
contours, _ = cv.findContours(thresh, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
# cv.drawContours(img, contours, -1, (0,0,255), 1)
# cv.imshow('Contours Drawn', img)


# Get bounding box of the largest contour
x, y, w, h = cv.boundingRect(contours[0])
# cv.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
# cv.imshow('Bounding box', img)
cropped_img = img[y:y + h, x:x + w]


cropped_img = cv.cvtColor(cropped_img, cv.COLOR_BGR2BGRA)
lower_white = np.array([200, 200, 200, 0], dtype=np.uint8)
upper_white = np.array([255, 255, 255, 255], dtype=np.uint8)

# Create a mask where white colors are detected
mask = cv.inRange(cropped_img, lower_white, upper_white)

# Set the alpha channel to 0 for the white areas
cropped_img[mask == 255] = [0, 0, 0, 0]


# cv.imshow('dst',dst)
fileName_new = 'Landing_Rate_Fig_PlaneAngle_0_Rotated.png'
cv.imwrite(os.path.join(BASEPATH,fileName_new),cropped_img)
cv.waitKey(0)



