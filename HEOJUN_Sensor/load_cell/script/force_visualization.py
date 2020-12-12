#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from load_cell_msgs.msg import LoadCellForceVector
from load_cell_msgs.msg import LoadCellForceArray
from load_cell_msgs.msg import LoadCellForce

import cv2
import sys
import math

force_array = LoadCellForceArray()
force_vector = LoadCellForceVector()
background_image_name = '/home/ubuntu/catkin_ws/src/HEOJUN/HEOJUN_Sensor/load_cell/image/heojun.jpg'
background_image = cv2.imread(background_image_name, cv2.IMREAD_COLOR)
image_width = background_image.shape[1]
image_height = background_image.shape[0]


def force2bar(force):
  bar_size = force

  return bar_size

def rosDown():
  rospy.loginfo('Force Visualization Done!')
  sys.exit(1)

def drawForce():
  global force_array, force_vector, background_image, image_width, image_height
 
  frame = background_image.copy()
  center_width = int(image_width / 2)
  center_height = int(image_height / 2)

  force_theta = (0.0, 3.0/2, 1.0/2, 1.0)
  base_pos = 50
  force_position = ((0, base_pos), (base_pos, 0), (-base_pos, 0), (0, -base_pos))

  force_values = force_array.data
  for force_value in force_values:
    sensor_id = force_value.id
    force = force_value.force

    theta = math.pi * force_theta[sensor_id]
    bar_size = force2bar(force)
    bar_start_x = force_position[sensor_id][0] + center_width
    bar_start_y = force_position[sensor_id][1] + center_height
    bar_end_x = int(bar_start_x + math.sin(theta) * bar_size)
    bar_end_y = int(bar_start_y - math.cos(theta) * bar_size)
    bar_start = (bar_start_x, bar_start_y)
    bar_end = (bar_end_x, bar_end_y)

    bar_percent = bar_size / image_width
    color = (int(255*bar_percent), int(255*(1-bar_percent)), 0)
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.line(frame, bar_start, bar_end, color, 10)
    cv2.putText(frame, str(sensor_id), bar_start, font, 0.5, (0, 100, 255), 2)

  x_vector = force_vector.x
  y_vector = force_vector.y
  vector_size = force_vector.force

  vector_start = (center_width, center_height)
  vector_end_x = vector_start[0] + int(x_vector * vector_size)
  vector_end_y = vector_start[1] - int(y_vector * vector_size)
  vector_end = (vector_end_x, vector_end_y)
  color = (0, 100, 255)
  cv2.arrowedLine(frame, vector_start, vector_end, color, 3)

  cv2.imshow('Force Visualization', frame)
  key = cv2.waitKey(1)
  if key == ord('q'):
    cv2.destroyAllWindows()
    rospy.signal_shutdown('Force Visualization Done!')

def arrayCallback(data):
  global force_array
  
  force_array = data

  drawForce()

def vectorCallback(data):
  global force_vector

  force_vector = data
  rospy.loginfo(force_vector)
  rospy.loginfo('===============================================')

def forceVisualization():
  rospy.init_node('force_visualization', anonymous=True)
 
  rospy.Subscriber('/load_cell/force_array', LoadCellForceArray, arrayCallback)
  rospy.Subscriber('/load_cell/force_vector', LoadCellForceVector, vectorCallback)

  rospy.spin()

if __name__ == '__main__':
 forceVisualization()
