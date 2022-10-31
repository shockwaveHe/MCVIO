#!/usr/bin/env python3

# Draw fisheye mask based on calibration result
# Henry Zhang <hzhang0407@gmail.com>

# Read in Kalibr camera calibration yaml result

import cv2
import os
import argparse
import yaml
import numpy as np

def main():
  parser = argparse.ArgumentParser(description='Draw the fisheye mask from Kalibr calibration output')
  parser.add_argument('yaml_path', type=str,
                      help='Kalibr yaml file path for the fisheye mask')
  parser.add_argument('--output_folder', default='./', type=str,
                      help='folder to output the fisheye mask')

  args = parser.parse_args()

  # read the yaml file
  with open(args.yaml_path, 'r') as file:
    cams = yaml.full_load(file)
    for cam_id in cams:
      # get image res and prinicple points
      cam = cams[cam_id]
      res = cam['resolution']
      intrinsics = cam['intrinsics']
      u0 = int(intrinsics[2])
      v0 = int(intrinsics[3])

      # draw the circle
      image = np.zeros((res[1], res[0]), np.uint8)
      cv2.circle(image, (u0, v0), int(max(res)/2), (255), -1)

      # draw subt ds led
      left_led = np.array([[145,868],[266,827],[405,875],[425,955],[454,800],[573,733],[810,727],[934,780],[975,871],[1145,815],[1193,819],[1280,960],[0,960],[145,868]],np.int32)

      # left_led = (left_led / 2).astype(np.int32)
      print(left_led)
      
      cv2.fillPoly(image, [left_led], (0,0,0))
      
      path ='/home/yao/visual_odometry/vins_laser_final_backup/src/config/sample.png'
  
      #Using cv2.imread() method
      img = cv2.imread(path)
  
      #Displaying the image
      cv2.imshow('sample', img)

      cv2.imshow('mask', image)
      cv2.waitKey(0)
      cv2.destroyAllWindows()

      # save image
      img_name = 'fisheye_mask_%s.png' %(cam_id)
      img_name = os.path.join(args.output_folder, img_name)
      cv2.imwrite(img_name, image)


if __name__ == "__main__":
  main()
