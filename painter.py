"""
Nao becomes a painter
"""

import sys, motion, almath, time, pickle, copy
import numpy as np
from PIL import Image
import cv2
from naoqi import ALProxy

IP = "127.0.0.1"
PORT = 9559
resolution = 2   #VGA
colorSpace = 11  #RGB
camIndex = 0 #0 is camera UP, 1 is camera DOWN
fps = 5


"""
Get an image from NAO. Display it and save it using PIL.
"""
def getImage():
  motionProxy = ALProxy("ALMotion", IP, PORT)

  #put head in start position
  effectorList = ["HeadPitch","HeadYaw"]
  headPitch = -0.0015759468078613281
  headYaw = 0.0
  fractionMaxSpeed = 0.2
  motionProxy.setAngles(effectorList[0], headPitch, fractionMaxSpeed)
  motionProxy.setAngles(effectorList[1], headYaw, fractionMaxSpeed)

  voice = ALProxy("ALTextToSpeech", IP, PORT)
  voice.say("I am taking the picture immediately.")

  time.sleep(3)

  camProxy = ALProxy("ALVideoDevice", IP, PORT)
  videoClient =camProxy.subscribeCamera("NAO_CAM", camIndex, resolution, colorSpace, fps)
  #Get a camera image
  # image[6] contains the image data passed as an array of ASCII chars
  naoImage = camProxy.getImageRemote(videoClient)

  # Now we work with the image returned and save it as a PNG using ImageDraw package

  #Get the image size and pixel array
  imageWidth = naoImage[0]
  imageHeight = naoImage[1]
  array = naoImage[6]

  # Create a PIL Image from our pixel array
  image = Image.fromstring("RGB", (imageWidth, imageHeight), array)
  # Save the image
  image.save("camImage.png")

  camProxy.unsubscribe(videoClient)

  # load the image
  original = cv2.imread("camImage.png")

  # apply erosion
  kernel = np.ones((2,2),np.uint8)
  originalErosion = cv2.erode(original, kernel, iterations =1)
  # converting the original image into grayscale
  originalGray = cv2.cvtColor(originalErosion, cv2.COLOR_BGR2GRAY)
  # reducing the noise in the image
  blur = cv2.GaussianBlur(originalGray,(3,3),0)

  # create a new image of the same size of the starting image
  height, width = originalGray.shape
  newimg = np.zeros((height, width, 3), np.uint8)

  img = Image.fromarray(newimg)
  img.save("newImage.png")

  # function of Canny edge detector - retrieving edges
  thresh = 175
  originalEdges = cv2.Canny(blur, thresh, thresh*2)

  cv2.imwrite("newImage.png", originalEdges)

  # extract contours
  contours, hierarchy = cv2.findContours(originalEdges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

  motionProxy.setStiffnesses("RArm", 1.0)

  #position Shoulder
  angleShoulderPitch = 0.26
  motionProxy.setAngles("RShoulderPitch", angleShoulderPitch, fractionMaxSpeed)

  #position Elbow
  angleShoulderRoll = -0.9
  motionProxy.setAngles("RShoulderRoll", angleShoulderRoll, fractionMaxSpeed)

  angleElbowRoll = 1.4818859100341797
  motionProxy.setAngles("RElbowRoll", angleElbowRoll, fractionMaxSpeed)

  time.sleep(1)

  # extract corner points
  pixels = cv2.cornerHarris(blur,2,3,0.04)
  pixels = cv2.dilate(pixels,None)
  original[pixels>0.01*pixels.max()]=[0,0,255]
  cv2.imshow('pixels',original)
  cv2.waitKey()

  # loop over the contours
  for cnt in contours:
    #approx = cv2.approxPolyDP(cnt,0.1*cv2.arcLength(cnt,True), True)
    nao_painting(pixels)

  voice.say("Get ready to admire my latest piece of art!")

"""
Draws the shape seen by NAO.
"""
def nao_painting(points):
  effector   = "RArm"
  frame = motion.FRAME_TORSO
  axisMask = almath.AXIS_MASK_X + almath.AXIS_MASK_Y + almath.AXIS_MASK_Z
  fractionSpeed = 0.5
  timeLists=[]

  points=pickle.load(open("schema.p","rb"))

  motionProxy = ALProxy("ALMotion", IP, PORT)
  handPosition = motionProxy.getPosition("RHand",frame,False)

  pointList = []
  for p in points:
    value = []
    newX = (float(p[1]/1850))*0.3
    newY = (-(float(p[0])/1600)*0.3)
    value.insert(0,newX)
    value.insert(1,newY)

    pointList.append(value)

  evolList = []
  for i in range(len(pointList)):
    evolPoint = []
    evolPoint.insert(0,(pointList[i][0])-(pointList[0][0]))
    evolPoint.insert(1,(pointList[i][1])-(pointList[0][1]))
    evolList.append(evolPoint)

  movementsList = []
  for i in evolList:
    newPosition = copy.copy(handPosition)
    newPosition[0] = newPosition[0] + (i[0])
    newPosition[1] = newPosition[1] + (i[1])
    movementsList.append(newPosition)

  #f = open("movements.txt","w")
  #for i in movementsList:
    #f.write(str(i)+"\n")
  #f.close()

  # create a circle manually
  # record movements
  # pickle.dump(movementsList,open("schema.p"),"wb")

  for i in range(len(movementsList)):
    timeLists.append(i+1)
  print timeLists

  motionProxy = ALProxy("ALMotion", IP, PORT)
  #motionProxy.positionInterpolations(effector, frame, movementsList, axisMask, timeLists)
  motionProxy.setPositions(effector, frame, movementsList[0], 0.5, axisMask)
  time.sleep(1)
  motionProxy.setPositions(effector, frame, movementsList[len(movementsList)/2], 0.5, axisMask)
  time.sleep(1)
  motionProxy.setPositions(effector, frame, movementsList[len(movementsList)-1], 0.5, axisMask)

def main():
  print("\nNao becomes a painter!\n")

  postureProxy = ALProxy("ALRobotPosture", IP, PORT)
  postureProxy.goToPosture("StandInit",0.5)

  getImage()

  motionProxy = ALProxy("ALMotion", IP, PORT)
  motionProxy.rest()

  print("The end.")

if __name__ == '__main__':
  main()
