from straight_line import drive_straight_for_distance, rotate, ANGLE_CALIBRATION, WHEEL_ROTATION_FOR_90_DEGREES

DISTANCE = 40 # in cm
ANG_VELOCITY = -120

if __name__ == "__main__":
  for _ in range(4):
    
    drive_straight_for_distance(DISTANCE, ANG_VELOCITY)
    rotate(WHEEL_ROTATION_FOR_90_DEGREES)
