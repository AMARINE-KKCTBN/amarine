from ast import Not
import vision.vision_lib as vision_lib

vision = vision_lib.hsv_detector()
while True:
    vision.detect_circle_object()
    coord_x = vision.get_circle_x()
    if coord_x < 0:
        coord_x = -1
    print("coord: " + str(coord_x))
        
    if not vision.show_image():
       break 