import cv2
import numpy as np
import json

class hsv_detector:
    def __init__(self, camera_height = 320, camera_width = 240, detect_contours_mode = False, radius_limiter = False, stabilizer_enabled = False, masking_enabled = False, record_enabled = False, visualization_enabled = False, vertical_limiter = False, horizontal_limiter = False):
        self.object_hsv_path = 'vision/object_hsv.json'
        self.field_hsv_path = 'vision/field_hsv.json'
        self.circle_params_path = 'vision/circle_params.json'
        self.transform_params_path = 'vision/transform_params.json'
        
        #init camera
        self.camera = cv2.VideoCapture(0)
        self.camera.set(cv2.CAP_PROP_FPS, 30)
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, camera_width)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, camera_height)
        self.camera_height = int(self.camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.camera_width = int(self.camera.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.camera_fps = int(self.camera.get(cv2.CAP_PROP_FPS))
        
        self.image_output = np.zeros((100,100,3), dtype=np.uint8)
        
        self.masking_enabled = masking_enabled
        self.visualization_enabled = visualization_enabled
        self.record_enabled = record_enabled
        self.stabilizer_enabled = stabilizer_enabled
        
        self.detect_contours_mode = detect_contours_mode

        self.vertical_limiter = vertical_limiter
        self.horizontal_limiter = horizontal_limiter
        self.radius_limiter = radius_limiter
        
        self.vertical_upper_limit = 0.5
        self.vertical_lower_limit = 0
        
        self.horizontal_upper_limit = 1
        self.horizontal_lower_limit = 0
        
        self.radius_upper_limit = 1
        self.radius_lower_limit = 0
        
        self.output_x = 0
        self.output_y = 0
        self.output_z = 0
        
        self.video_codec = cv2.VideoWriter_fourcc(*'mp4v') ##(*'XVID')
        self.video_file = "Recorded video.mp4"
        self.record_video = cv2.VideoWriter(self.video_file, self.video_codec, self.camera_fps, (self.camera_width, self.camera_height))
        
        self.read_params()
        
        #morph kernel param
        self.morph_kernel = np.ones((self.transform_params['object_morph_kernel'],self.transform_params['object_morph_kernel']),"uint8")
        
        #object hsv filter params
        self.object_LowerRegion = np.array([self.object_hsv['H Lower'],self.object_hsv['S Lower'],self.object_hsv['V Lower']],np.uint8)
        self.object_upperRegion = np.array([self.object_hsv['H Higher'],self.object_hsv['S Higher'],self.object_hsv['V Higher']],np.uint8)
        
        #field hsv filter params
        self.field_LowerRegion = np.array([self.field_hsv['H Lower'],self.field_hsv['S Lower'],self.field_hsv['V Lower']],np.uint8)
        self.field_upperRegion = np.array([self.field_hsv['H Higher'],self.field_hsv['S Higher'],self.field_hsv['V Higher']],np.uint8)
        
    def read_params(self):        
        with open(self.object_hsv_path, 'r') as openfile:
            self.object_hsv = json.load(openfile)
        with open(self.field_hsv_path, 'r') as openfile:
            self.field_hsv = json.load(openfile)
        with open(self.circle_params_path, 'r') as openfile:
            self.circle_params = json.load(openfile)
        with open(self.transform_params_path, 'r') as openfile:
            self.transform_params = json.load(openfile)
    
    def read_camera(self):
        _,self.image_bgr = self.camera.read()
        #self.image_bgr = cv2.flip(self.image_bgr,1)

        self.image_hsv = cv2.cvtColor(self.image_bgr, cv2.COLOR_BGR2HSV)
        
    def show_image(self):
        cv2.imshow("Base Image", self.image_bgr)
        cv2.imshow("Output Image", self.image_output)
        if self.masking_enabled:
            cv2.imshow("Field-mask Image", self.field_mask)
            # cv2.imshow("Output with Field-masking", self.object_with_mask)
            
        if cv2.waitKey(10) & 0xFF == ord('q'):
            self.camera.release()
            cv2.destroyAllWindows()
            return False
        else:
            return True
    
    def filter_and_morph_image(self):
        self.object_mask = self.image_hsv
        
        #filter image
        if self.object_hsv['H Lower'] < self.object_hsv['H Higher']:
                #normal
            object_LowerRegion = np.array([self.object_hsv['H Lower'],self.object_hsv['S Lower'],self.object_hsv['V Lower']],np.uint8)
            object_upperRegion = np.array([self.object_hsv['H Higher'],self.object_hsv['S Higher'],self.object_hsv['V Higher']],np.uint8)
            self.object_mask = cv2.inRange(self.object_mask,object_LowerRegion,object_upperRegion)
        else:
            #h lower > h higher
            object_LowerRegion = np.array([0,self.object_hsv['S Lower'],self.object_hsv['V Lower']],np.uint8)
            object_upperRegion = np.array([self.object_hsv['H Higher'],self.object_hsv['S Higher'],self.object_hsv['V Higher']],np.uint8)
            object_hue_lower = cv2.inRange(self.object_mask,object_LowerRegion,object_upperRegion)
            
            object_LowerRegion = np.array([self.object_hsv['H Lower'],self.object_hsv['S Lower'],self.object_hsv['V Lower']],np.uint8)
            object_upperRegion = np.array([179,self.object_hsv['S Higher'],self.object_hsv['V Higher']],np.uint8)
            object_hue_higher = cv2.inRange(self.object_mask,object_LowerRegion,object_upperRegion)
            
            self.object_mask = cv2.bitwise_or(object_hue_lower, object_hue_higher)
        
        #morphological operations
        self.object_mask = cv2.morphologyEx(self.object_mask,cv2.MORPH_OPEN, self.morph_kernel)
        self.object_mask = cv2.morphologyEx(self.object_mask,cv2.MORPH_CLOSE, self.morph_kernel)
        
        #sub: dilate-erode
        # self.object_mask = cv2.dilate(self.object_mask, self.morph_kernel,iterations=1)
        # self.object_mask = cv2.erode(self.object_mask, self.morph_kernel, iterations=2)
        
    def field_masking(self):
        self.field_mask = self.image_hsv
        
        #filter image
        self.field_mask = cv2.inRange(self.field_mask, self.field_LowerRegion, self.field_upperRegion)
        
        #find contours
        contours, hierarchy = cv2.findContours(self.field_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        #convex hull
        hull_list = []
        for i in range(len(contours)):
            hull = cv2.convexHull(contours[i])
            hull_list.append(hull)
        
        #enclosed hull   
        enclosed_mask = np.zeros(self.image_bgr.shape, np.uint8)
        enclosed_mask = cv2.drawContours(enclosed_mask, hull_list, -1, (255,255,255), -1)
        
        self.field_mask = cv2.bitwise_and(self.image_bgr, enclosed_mask)
        
        self.object_mask = cv2.bitwise_and(self.image_bgr, enclosed_mask, mask = self.object_mask)

    #circle detector (without field masking)
    def detect_circle_object(self):        
        self.object_edges = cv2.Canny(self.object_mask, self.transform_params['object_canny_param1'], self.transform_params['object_canny_param2'])

        circles = cv2.HoughCircles(self.object_edges, cv2.HOUGH_GRADIENT, self.circle_params['param3'], self.circle_params['param4'],
                                        param1=self.circle_params['param1'], param2=self.circle_params['param2'], minRadius=self.circle_params['min_radius'], maxRadius=self.circle_params['max_radius'])

        self.circle_x = -1
        self.circle_y = -1
        self.circle_z = -1
        
        coord_obj = None
        
        temp_x = -1
        temp_y = -1
        biggest_radius = 0
        if circles is not None:
            circles = np.uint16(np.around(circles))
            # print(circles)
            for i in circles[0, :]:
                # draw the outer circle
                # draw the center of the circle
                # save output to variables
                
                #draw all circle
                if self.visualization_enabled:
                    cv2.circle(self.image_output, (i[0], i[1]), i[2], (0, 0, 255), 2)
                    cv2.circle(self.image_output, (i[0], i[1]), 2, (255, 0, 0), 3)
                
                #choose the biggest circle
                if self.check_radius_limit(i[2]) and i[2] > biggest_radius:
                    temp_x = round(i[0] / self.camera_width * 2 - 1, 3)
                    temp_y = round(i[1] / self.camera_height * 2 - 1, 3)
                    
                    #choose the circles within limit
                    if  self.check_vertical_limit(temp_y) and self.check_horizontal_limit(temp_x):
                        coord_obj = i
                        biggest_radius = i[2]
                    else:
                        temp_x = -1
                        temp_y = -1
                
            #published data, -1 -> 1
            self.circle_x = temp_x
            self.circle_y = temp_y
            self.circle_z = biggest_radius
            #draw choosen circle
            if coord_obj is not None:
                cv2.circle(self.image_output, (coord_obj[0], coord_obj[1]), coord_obj[2], (0, 255, 0), 2)
                cv2.circle(self.image_output, (coord_obj[0], coord_obj[1]), 2, (255, 0, 0), 3)

    def detect_contours(self):
        _,object_contours,h=cv2.findContours(self.object_mask.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
        cv2.drawContours(self.image_output,object_contours,-1,(255,0,0),3)
        for i in range(len(object_contours)):
            object_moments = cv2.moments(object_contours[i])
            x,y,w,h=cv2.boundingRect(object_contours[i])
            if object_moments["m00"] != 0:
                self.circle_x = int(object_moments["m10"] / object_moments["m00"])
                self.circle_y = int(object_moments["m01"] / object_moments["m00"])
            else:
                self.circle_x = -1
                self.circle_y = -1
            cv2.rectangle(self.image_output,(x,y),(x+w,y+h),(0,0,255), 2)
            cv2.circle(self.image_output, (self.circle_x, self.circle_y), 5, (255, 255, 255), -1)
            cv2.putText(self.image_output, str(i+1),(x,y+h),cv2.FONT_HERSHEY_SIMPLEX,1.0,(0,255,255))

    def stabilizer(self):
        if self.circle_x != -1:
            if self.output_x != -1:
                self.output_x = (self.output_x + self.circle_x) / 2
                self.output_y = (self.output_y + self.circle_y) / 2
            else:
                self.output_x = self.circle_x
                self.output_y = self.circle_y
        else:
            self.output_x = self.circle_x
            self.output_y = self.circle_y
    
    def check_radius_limit(self, temp_rad):
        temp_rad = round(temp_rad / self.camera_height * 4, 3)
        return temp_rad > self.radius_lower_limit and temp_rad < self.radius_upper_limit or not self.radius_limiter
    
    def check_horizontal_limit(self, temp_x):
        return temp_x > self.horizontal_lower_limit and temp_x < self.horizontal_upper_limit or not self.horizontal_limiter
                
    def check_vertical_limit(self, temp_y):
        return temp_y > self.vertical_lower_limit and temp_y < self.vertical_upper_limit or not self.vertical_limiter
    
    def draw_line(self, start_x, start_y, end_x, end_y):
        start_point_x = int((start_x + 1) / 2 * self.camera_width)
        start_point_y = int((start_y + 1) / 2 * self.camera_height)
        end_point_x = int((end_x + 1) / 2 * self.camera_width)
        end_point_y = int((end_y + 1) / 2 * self.camera_height)
        cv2.line(self.image_output, (start_point_x, start_point_y), (end_point_x, end_point_y), (255, 255, 255), 1)
    
    def draw_green_line(self, start_x, start_y, end_x, end_y):
        start_point_x = int((start_x + 1) / 2 * self.camera_width)
        start_point_y = int((start_y + 1) / 2 * self.camera_height)
        end_point_x = int((end_x + 1) / 2 * self.camera_width)
        end_point_y = int((end_y + 1) / 2 * self.camera_height)
        cv2.line(self.image_output, (start_point_x, start_point_y), (end_point_x, end_point_y), (0, 255, 0), 3)
        
    def line_visualization(self):
        #vertical lower limit
        self.draw_line(-1, self.vertical_lower_limit, 1, self.vertical_lower_limit)
        #vertical upper limit
        self.draw_line(-1, self.vertical_upper_limit, 1, self.vertical_upper_limit)
        
        #horizontal left limit
        self.draw_line(self.horizontal_lower_limit, -1, self.horizontal_lower_limit, 1)
        #horizontal right limit
        self.draw_line(self.horizontal_upper_limit, -1, self.horizontal_upper_limit, 1)
        #horizontal center reference
        self.draw_line(self.horizontal_upper_limit/2, -1, self.horizontal_upper_limit/2, 1)
        
        #draw coord_x
        # if self.circle_x is not -1:
        #     self.draw_green_line(self.circle_x, 0, self.circle_x, 0.5)
        if self.output_x != -1:
            self.draw_green_line(self.output_x, 0, self.output_x, 0.5)
    
    def main_process(self):
        self.read_camera()
        self.filter_and_morph_image()
        
        self.image_output=cv2.bitwise_and(self.image_bgr, self.image_bgr, mask = self.object_mask)
        
        if self.masking_enabled:
            self.field_masking()
        
        if self.detect_contours_mode:
            self.detect_contours()
        else:
            self.detect_circle_object()
        
        if self.stabilizer_enabled:
            self.stabilizer()
        
        if self.visualization_enabled:
            self.line_visualization()
        
        if self.record_enabled:
            self.record_video.write(self.image_output)
    
    def enable_vertical_limiter(self, lower_limit = 0, upper_limit = 0.5):
        self.vertical_limiter = True
        self.vertical_lower_limit = lower_limit
        self.vertical_upper_limit = upper_limit
    
    def enable_horizontal_limiter(self, lower_limit = 0, upper_limit = 1):
        self.horizontal_limiter = True
        self.horizontal_lower_limit = lower_limit
        self.horizontal_upper_limit = upper_limit
    
    def enable_radius_limiter(self, lower_limit = 0, upper_limit = 1):
        self.radius_limiter = True
        self.radius_lower_limit = lower_limit
        self.radius_upper_limit = upper_limit
    
    def visualize(self):
        self.visualization_enabled = True
    
    def stabilize(self):
        self.stabilizer_enabled = True
    
    def record(self):
        self.record_enabled = True

    def enable_contours_mode(self):
        self.detect_contours_mode = True
                  
    def get_circle_coord(self):
        return self.circle_x, self.circle_y, self.circle_z