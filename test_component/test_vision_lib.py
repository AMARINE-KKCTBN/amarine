from ast import Not
import vision.vision_lib as vision_lib

offset_x = 0.5
vision = vision_lib.hsv_detector(camera_height=9999, camera_width=9999)
while True:
    vision.detect_circle_object()
    coord_x = vision.get_circle_x()
    if coord_x < 0:
        coord_x = -1
    else:
        coord_x -= offset_x
    print("coord: " + str(round(coord_x, 3)))
        
    if not vision.show_image():
       break 