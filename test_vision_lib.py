import vision.vision_lib as vision_lib

def simulated_servo(temp_x):
    range_deg = 60
    range_output = 1
    offset_deg = 0
    output2deg = temp_x / range_output * range_deg
    servo = output2deg + offset_deg
    return servo

vision = vision_lib.hsv_detector(#camera_height=9999, camera_width=9999, 
                                 #image source: 0 for camera index, or "file_name.mp4" to play from video
                                #  image_source="Recorded video8.mp4"
                                 image_source=0
                                 )
vision.set_offset_x(-0.75)
vision.enable_vertical_limiter(-0.3, 0.5)
vision.enable_horizontal_limiter(-1, -0.2)
vision.enable_radius_limiter(0.2, 0.7)
# vision.enable_horizontal_limiter(0.2, 1)
# vision.enable_radius_limiter(0.2, 0.5)
vision.enable_averaging()
vision.enable_contours_mode()
vision.resize_input_image(640, 480)
vision.visualize()
vision.stabilize()
# vision.record(record_output=True)
wait_count = 0
wait_limit = 10
while True:
    vision.main_process()
    # if coord_x < 0:
    #     coord_x = -1
    # else:
    if vision.object_detected():
        coord_x, coord_y, coord_z = vision.get_average_coord()
        # coord_x -= vision.get_offset_x()
        text = "coord: " + str(vision.get_average_coord()) + " | output: " + str(round(simulated_servo(coord_x), 3))
        vision.put_text(text)
        print(text)
        
    if not vision.show_image():
       break  