#!/usr/bin/python2.7

'''
File: landmark_detection_pr2.py
Project: Stage UFPE Recife, BR, 2019-2020
File Created: Tuesday, 21st April 2020 9:44:53 am
-----
University: Universite Paul Sabatier - Toulouse (UPPSITECH - SRI)
Author: Benoit TRINDADE (benoit.trindade31@gmail.com)
-----
Last Modified: Sunday, 23rd August 2020 6:54:43 pm
Modified By: Benoit TRINDADE (benoit.trindade31@gmail.com>)
-----
Copyright a choisir - 2020 Etudiant
-----
HISTORY:
Date       	By	Comments
-----------	---	---------------------------------------------------------

28-04-202020	BT	Using fixed depth ( no mesure ) & changing focal and pixel size

21-04-202020	BT	Reading raw stream out from l_forearm_cam
'''






import roslib
import struct
import sys
import rospy
import cv2
import numpy as np
import ros_numpy
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from cv_bridge import CvBridge, CvBridgeError

#from pr2_forearm_vision.msg import visualFeatures
from pr2_forearm_vison.msg import visualFeatures

class image_processing:

  def __init__(self):


     # Bridge to convert ROS message to openCV
      self.bridge = CvBridge()

      # QR detector
      self.qrDecoder = cv2.QRCodeDetector()

      # Init the depth array
      self.l_zArray = np.ones((480,640))
      self.r_zArray = np.ones((480,640))

      # Subscriber to the camera image
      self.l_image_sub = rospy.Subscriber("/l_forearm_cam/image_color",Image,self.l_callBackQR)
      self.r_image_sub = rospy.Subscriber("/r_forearm_cam/image_color",Image,self.r_callBackQR)

      # Subscriber to the camera pointcloud
      self.pc_sub = rospy.Subscriber("/xtion/depth_registered/points", PointCloud2, self.callbackPC)

      # Publisher of the visual features coordinates
      self.l_vf_pub = rospy.Publisher("l_visual_features", visualFeatures, queue_size=1)
      self.r_vf_pub = rospy.Publisher("r_visual_features", visualFeatures, queue_size=1)



  def callbackPC(self,msg):
    pc = ros_numpy.numpify(msg)

    
     ### self.zArray = pc['z']

    
      # points=np.zeros((pc.shape[0], pc.shape[1] ,3))
      # im=np.zeros((pc.shape[0], pc.shape[1]))
      # points[:,:,0]=pc['x']
      # points[:,:,1]=pc['y']
      # points[:,:,2]=pc['z']
      # im=pc['rgb']
      #
      # cv_im = np.zeros((pc.shape[0], pc.shape[1],3))
      #
      # for idx1 in range(0,im.shape[0]):
      #     for idx2 in range(0,im.shape[1]):
      #         bgra = struct.unpack('BBBB', struct.pack('I', im[idx1,idx2]))
      #         cv_im[idx1, idx2, 0] = bgra[0]
      #         cv_im[idx1, idx2, 1] = bgra[1]
      #         cv_im[idx1, idx2, 2] = bgra[2]
      # # print(rgb)
      #
      # cv2.imshow("raw", cv_imimage = cv2.imread('C:/Users/N/Desktop/testQRCode.png')

  def l_callBackQR(self, msg):

    try:
      cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
      print(e)

    bbox = np.array([])
    data = False
    try :
      #cv2.imshow("raw", cv_image)
      #cv2.waitKey(3)
      ### note : always stop texture projector with rosrun rqt_reconfigure rqt_reconfigure###
      data,bbox = self.qrDecoder.detect(cv_image)
      # print "data : " + str(data)
      # print "\n " + str(bbox)
      if data == False : return 

    except cv2.error as e:
      print "Le QR code n'est pas reconnu dans l'image ou n'est pas valide"

    # Convert to xy camera frame
    # 0.0000019 m is the size of a pixel
    # focal lenght 1 mm


#  Coordinates (xim,yim) of image point in pixel units related to coordinates (x,y)
#   of same point in camera ref frame by:

#   x = - (x_image - centreImage_x) * size_pixel_x
#   y = - (y_image - centreImage_y) * size_pixel_y

#   where (ox,oy) is the image center and sx, sy denote size of pixel
    estimate_focal = 0.001 # 1mm ??? 
    # size_pixel_x = ???? 

    x1 =  (bbox[0][0][1] - 240)*estimate_focal
    y1 = -(bbox[0][0][0] - 320)*estimate_focal
    x2 =  (bbox[1][0][1] - 240)*estimate_focal
    y2 = -(bbox[1][0][0] - 320)*estimate_focal
    x3 =  (bbox[2][0][1] - 240)*estimate_focal # 0.003125
    y3 = -(bbox[2][0][0] - 320)*estimate_focal
    x4 =  (bbox[3][0][1] - 240)*estimate_focal
    y4 = -(bbox[3][0][0] - 320)*estimate_focal

    # Publish the visual features
    msg = visualFeatures()
    msg.x1 = x1
    msg.y1 = y1
    msg.x2 = x2
    msg.y2 = y2
    msg.x3 = x3
    msg.y3 = y3
    msg.x4 = x4
    msg.y4 = y4
    msg.z1 = self.l_zArray[0,0]
    msg.z2 = self.l_zArray[0,0]
    msg.z3 = self.l_zArray[0,0]
    msg.z4 = self.l_zArray[0,0]

    try:
      self.l_vf_pub.publish(msg)
      #print " MSG... lPublished"
    except CvBridgeError as e:
      print(e)
      
    #(rows,cols,channels) = cv_image.shape
    cv2.circle(cv_image,(bbox[0][0][0], bbox[0][0][1]), 5, (255,255,50), 2)
    cv2.circle(cv_image,(bbox[1][0][0], bbox[1][0][1]), 5, (255,255,50), 2)
    cv2.circle(cv_image,(bbox[2][0][0], bbox[2][0][1]), 5, (255,255,50), 2)
    cv2.circle(cv_image,(bbox[3][0][0], bbox[3][0][1]), 5, (255,255,50), 2)


    # x1 = int( (x1/estimate_focal)+240)
    # y1 = int(-(y1/estimate_focal)+320)
    # x2 = int( (x2/estimate_focal)+240)
    # y2 = int(-(y2/estimate_focal)+320)
    # x3 = int( (x3/estimate_focal)+240)
    # y3 = int(-(y3/estimate_focal)+320)
    # x4 = int( (x4/estimate_focal)+240)
    # y4 = int(-(y4/estimate_focal)+320)
    # print(x1,y1,x2,y2,x3,y3,x4,y4)
    VF=[-0.11600000000000001, -0.17998931884765626, 0.170086181640625, -0.18101098632812501, 0.17301016235351563, 0.107, -0.11600000000000001, 0.107]
    # VF=[-116, -180, 170, -181, 173, 107, -116, 107]
    # VF=[-0.11959091186523438, -0.20100000000000001, 0.17315881347656251, -0.20100000000000001, 0.16807647705078124, 0.087000000000000008, -0.11304545593261719, 0.087000000000000008]
    x1 = int( (VF[0]/estimate_focal)+240)
    y1 = int(-(VF[1]/estimate_focal)+320)
    x2 = int( (VF[2]/estimate_focal)+240)
    y2 = int(-(VF[3]/estimate_focal)+320)
    x3 = int( (VF[4]/estimate_focal)+240)
    y3 = int(-(VF[5]/estimate_focal)+320)
    x4 = int( (VF[6]/estimate_focal)+240)
    y4 = int(-(VF[7]/estimate_focal)+320)


    # cv2.circle(image, center_coordinates, radius, color, thickness)
    # coord of the vf that we want to reach ! 
    cv2.circle(cv_image,(y1, x1), 5, (0,0,255), 2)
    cv2.circle(cv_image,(y2, x2), 5, (0,0,255), 2)
    cv2.circle(cv_image,(y3, x3), 5, (0,0,255), 2)
    cv2.circle(cv_image,(y4, x4), 5, (0,0,255), 2)

    cv2.imshow("raw", cv_image)
    cv2.waitKey(3)


  def r_callBackQR(self, msg):

    try:
      cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
      print(e)

    bbox = np.array([])
    data = False
    try :

      ### note : always stop texture projector with rosrun rqt_reconfigure rqt_reconfigure###
      data,bbox = self.qrDecoder.detect(cv_image)
      # print "data : " + str(data)
      # print "\n " + str(bbox)
      if data == False: return 

    except cv2.error as e:
      print "Le QR code n'est pas reconnu dans l'image ou n'est pas valide"

    estimate_focal = 0.001 # 1mm ??? 

    x1 =  (bbox[0][0][1] - 240)*estimate_focal
    y1 = -(bbox[0][0][0] - 320)*estimate_focal
    x2 =  (bbox[1][0][1] - 240)*estimate_focal
    y2 = -(bbox[1][0][0] - 320)*estimate_focal
    x3 =  (bbox[2][0][1] - 240)*estimate_focal # 0.003125
    y3 = -(bbox[2][0][0] - 320)*estimate_focal
    x4 =  (bbox[3][0][1] - 240)*estimate_focal
    y4 = -(bbox[3][0][0] - 320)*estimate_focal

    # Publish the visual feaatures
    msg = visualFeatures()
    msg.x1 = x1
    msg.y1 = y1
    msg.x2 = x2
    msg.y2 = y2
    msg.x3 = x3
    msg.y3 = y3
    msg.x4 = x4
    msg.y4 = y4
    msg.z1 = self.l_zArray[int(bbox[0][0][1]), int(bbox[0][0][0])]
    msg.z2 = self.l_zArray[int(bbox[1][0][1]), int(bbox[1][0][0])]
    msg.z3 = self.l_zArray[int(bbox[2][0][1]), int(bbox[2][0][0])]
    msg.z4 = self.l_zArray[int(bbox[3][0][1]), int(bbox[3][0][0])]

    try:
      self.r_vf_pub.publish(msg)
    except CvBridgeError as e:
      print(e)

def main(args):

  # Init image processing
  ic = image_processing()
  rospy.init_node('landmarkDetector', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
