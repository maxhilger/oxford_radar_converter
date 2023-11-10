#!/usr/bin/env python

import argparse
import os
from radar import load_radar, radar_polar_to_cartesian
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import rospy
import csv
from radar_io import ProcessFrame,signal_handler,BagWriter
from transform import build_se3_transform
import sys, signal
from cv_bridge import CvBridge, CvBridgeError

from decimal import *



parser = argparse.ArgumentParser(description='Play back radar data from a given directory')

parser.add_argument('dir', type=str, help='Directory containing radar data.')

args = parser.parse_args()
signal.signal(signal.SIGINT, signal_handler)


timestamps_path = os.path.join(os.path.join(args.dir,os.pardir, 'radar.timestamps'))
splitted_dir=args.dir.split("/")
bag_name=splitted_dir[-2]

bag_path = os.path.join(args.dir, bag_name)+".bag"
bw = BagWriter(bag_path)

if not os.path.isfile(timestamps_path):
    raise IOError("Could not find timestamps file")
#gt_path = os.path.join(os.path.join(args.dir, os.pardir), '/gt/radar_odometry.csv')
gt_path = os.path.join(args.dir, os.pardir)
#print (gt_path)
gt_path = os.path.join(gt_path,'gt/radar_odometry.csv')
print ("Loading csv at: "+gt_path)
if not os.path.isfile(gt_path):
    raise IOError("Could not find gt file")
ins_path = os.path.join(args.dir, os.pardir)
ins_path = os.path.join(ins_path,'gps/ins.csv')
if not os.path.isfile(ins_path):
    raise IOError("Could not find ins file")

# Cartesian Visualsation Setup
# Resolution of the cartesian form of the radar scan in metres per pixel
cart_resolution = .25
# Cartesian visualisation size (used for both height and width)
cart_pixel_width = 501  # pixels
interpolate_crossover = True

title = "Radar Visualisation Example"

rospy.init_node('radar_publisher', anonymous=True)
br = CvBridge()

pub_cart = rospy.Publisher('/Navtech/Cartesian', Image,queue_size=10)
pub_polar = rospy.Publisher('/Navtech/Polar', Image,queue_size=10)


radar_timestamps = np.loadtxt(timestamps_path, delimiter=' ', usecols=[0], dtype=np.int64)
GtPoseStamps=[]
InsMessages=[]
stamp = rospy.Time.now()
with open(gt_path, newline='') as csv_file: #Read radar csv file
    csv_reader = csv.reader(csv_file, delimiter=',')
    line_count = 0
    for row in csv_reader:
        GtPoseStamps.append(row)

with open(ins_path, newline='') as csv_file: #Read radar csv file
    csv_reader = csv.reader(csv_file, delimiter=',')
    line_count = 0
    for row in csv_reader:
        InsMessages.append(row)

for i in range(1, len(InsMessages)):
    curr_row = InsMessages[i]
    stamp = rospy.Time.from_sec(int(curr_row[0])/1000000)
    rpy = [float(curr_row[-3]), float(curr_row[-2]), float(curr_row[-1])]
    bw.WriteIns(rpy, stamp, '/imu')


current_frame = 0
pose_init = [0,0,0,0,0,0]
Tpose = build_se3_transform(pose_init)
curr_row = GtPoseStamps[1]
#stamp = rospy.Time.from_sec(int(curr_row[1])/1000000)
for radar_timestamp in radar_timestamps:


    if current_frame == len(GtPoseStamps) :
        break

    filename = os.path.join(args.dir, str(radar_timestamp) + '.png')
    if not os.path.isfile(filename):
        raise FileNotFoundError("Could not find radar example: {}".format(filename))

    timestamps, azimuths, valid, fft_data, radar_resolution = load_radar(filename)

    if current_frame == 0 :
        curr_inc=[0,0,0,0,0,0]
        curr_row = GtPoseStamps[current_frame+1]
        stamp = rospy.Time.from_sec(int(curr_row[1])/1000000)
    else:
        curr_row = GtPoseStamps[current_frame]
        stamp = rospy.Time.from_sec(int(curr_row[0])/1000000)
        curr_inc = [Decimal(curr_row[2]), Decimal(curr_row[3]), 0, 0, 0, Decimal(curr_row[7])]

    #stamp_ros = rospy.get_rostime()
    stamp_ros = stamp
    #print("scan: ")
    print("stamp: "+str(radar_timestamp))
    print("current_frame: "+str(current_frame))
    Tpose,tf_transform = ProcessFrame(curr_inc, Tpose, curr_inc, stamp_ros) ## read input transormation and perform fwdkinematics

    fft_data = (255*fft_data)
    img_int = fft_data.astype(np.uint8)

    height = fft_data.shape[0]
    width = fft_data.shape[1]
    msg_image_polar = br.cv2_to_imgmsg(img_int)

    msg_image_polar.header.stamp = stamp_ros
    #bw.WriteImage(msg_image_polar, stamp_ros, '/Navtech/Polar')
    bw.WritePointCloud(img_int, stamp_ros, '/Navtech/PointCloud')
    bw.WriteTf(tf_transform, stamp_ros)
                #   vis = cv2.hconcat( cart_img)

                #    cv2.imshow(title, vis * 2.)  # The data is doubled to improve visualisation
    #cv2.imshow(title, vis * 2.)  # The data is doubled to improve visualisation
    cv2.imshow(title, img_int)  # The data is doubled to improve visualisation

    #cv2.imshow(title, img_int)  # The data is doubled to improve visualisation
    #cv2.waitKey(1)
    current_frame+=1


                #bw.Close()
