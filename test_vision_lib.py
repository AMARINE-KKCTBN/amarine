import vision.vision_lib as vision_lib

def simulated_servo(temp_x):
    range_deg = 60
    range_output = 1
    offset_deg = 90
    output2deg = temp_x / range_output * range_deg
    servo = output2deg + offset_deg
    return servo

offset_x = 0.5
vision = vision_lib.hsv_detector(
    camera_height=9999, camera_width=9999, 
                                 #image source: 0 for camera index, or "file_name.mp4" to play from video
                                #  image_source="Recorded video.mp4"
                                 image_source=0
                                 )
# vision.enable_vertical_limiter(0, 0.5)
# vision.enable_horizontal_limiter(0, 1)
# vision.enable_radius_limiter(0.3, 0.5)
vision.enable_averaging()
vision.enable_contours_mode()
vision.visualize()
vision.stabilize()
vision.record(record_output=True)
while True:
    vision.main_process()
    coord_x, coord_y, coord_z = vision.get_circle_coord()
    # if coord_x < 0:
    #     coord_x = -1
    # else:
    #     coord_x -= offset_x
    text = "coord: " + str(vision.get_circle_coord()) + " | output: " + str(round(simulated_servo(coord_x), 3))
    vision.put_text(text)
    print(text)
        
    if not vision.show_image():
       break  