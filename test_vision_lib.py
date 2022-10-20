from ast import Not
import vision.vision_lib as vision_lib

vision = vision_lib.hsv_detector()
# vision.read_params()
while True:
    vision.read_camera()
    vision.detect_circle_object()
    if not vision.show_image():
       break 