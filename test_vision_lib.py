from ast import Not
import vision.vision_lib as vision_lib

vision = vision_lib.hsv_detector()
while True:
    vision.read_camera()
    vision.detect_circle_object()
    coord_x = vision.get_circle_x()
    if coord_x > 0:
        print("coord: " + str(coord_x))
    else:
        print("coord: " + str(-1))
        
    if not vision.show_image():
       break 