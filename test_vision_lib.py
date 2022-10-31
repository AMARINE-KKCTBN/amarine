from ast import Not
import vision.vision_lib as vision_lib

offset_x = 0.5
vision = vision_lib.hsv_detector()
while True:
    vision.detect_circle_object()
    coord_x, coord_y, coord_z = vision.get_circle_coord()
    if coord_x < 0:
        coord_x = -1
    else:
        coord_x -= offset_x
    print("coord: " + str(vision.get_circle_coord()))
        
    if not vision.show_image():
       break 