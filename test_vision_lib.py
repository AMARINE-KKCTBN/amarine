import vision.vision_lib as vision_lib

offset_x = 0.5
vision = vision_lib.hsv_detector(camera_height=9999, camera_width=9999)
vision.enable_vertical_limiter(0, 0.5)
vision.enable_horizontal_limiter(0, 1)
vision.visualize()
vision.stabilize()
vision.record()
while True:
    vision.main_process()
    coord_x, coord_y, coord_z = vision.get_circle_coord()
    if coord_x < 0:
        coord_x = -1
    else:
        coord_x -= offset_x
    print("coord: " + str(vision.get_circle_coord()) + " | output: " + str(coord_x))
        
    if not vision.show_image():
       break 