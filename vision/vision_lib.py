import cv2
import numpy as np
import json
from os import path




class hsv_detector:
    def __init__(self):
        self.object_hsv_path = 'vision/object_hsv.json'
        self.field_hsv_path = 'vision/field_hsv.json'
        self.circle_params_path = 'vision/circle_params.json'
        self.transform_params_path = 'vision/transform_params.json'
        
        #init camera
        self.camera_height = 240
        self.camera_width = 320
        self.camera = cv2.VideoCapture(0)
        self.camera.set(cv2.CAP_PROP_FPS, 30)
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.camera_width)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.camera_height)
        
        self.image_output = np.zeros((100,100,3), dtype=np.uint8)
        
        self.read_params()
        
        #morph kernel param
        self.morph_kernel = np.ones((self.transform_params['object_morph_kernel'],self.transform_params['object_morph_kernel']),"uint8")
        
        #object hsv filter params
        self.object_LowerRegion = np.array([self.object_hsv['H Lower'],self.object_hsv['S Lower'],self.object_hsv['V Lower']],np.uint8)
        self.object_upperRegion = np.array([self.object_hsv['H Higher'],self.object_hsv['S Higher'],self.object_hsv['V Higher']],np.uint8)
        
        

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
        
        if cv2.waitKey(10) & 0xFF == ord('q'):
            self.camera.release()
            cv2.destroyAllWindows()
            return False
        else:
            return True
    
    def morph_and_filter_image(self):
        self.object_mask = self.image_hsv
        
        #morphological operations
        self.object_mask = cv2.morphologyEx(self.object_mask,cv2.MORPH_OPEN, self.morph_kernel)
        self.object_mask = cv2.morphologyEx(self.object_mask,cv2.MORPH_CLOSE, self.morph_kernel)
        
        #sub: dilate-erode
        # self.object_mask = cv2.dilate(self.object_mask, self.morph_kernel,iterations=1)
        # self.object_mask = cv2.erode(self.object_mask, self.morph_kernel, iterations=2)
        
        #filter image
        self.object_mask = cv2.inRange(self.object_mask, self.object_LowerRegion, self.object_upperRegion)

    #circle detector (without field masking)
    def detect_circle_object(self): 
        self.morph_and_filter_image()
        
        self.object_edges = cv2.Canny(self.object_mask, self.transform_params['object_canny_param1'], self.transform_params['object_canny_param2'])

        self.circles = cv2.HoughCircles(self.object_edges, cv2.HOUGH_GRADIENT, self.circle_params['param3'], self.circle_params['param4'],
                                        param1=self.circle_params['param1'], param2=self.circle_params['param2'], minRadius=self.circle_params['min_radius'], maxRadius=self.circle_params['max_radius'])
        
        self.image_output=cv2.bitwise_and(self.image_bgr, self.image_bgr, mask = self.object_mask)

        self.circle_x = -1
        self.circle_y = -1
        self.circle_z = -1
        
        biggest_radius = 0
        if self.circles is not None:
            self.circles = np.uint16(np.around(self.circles))
            # print(self.circles)
            for i in self.circles[0, :]:
                # draw the outer circle
                cv2.circle(self.image_output, (i[0], i[1]), i[2], (0, 255, 0), 2)
                # draw the center of the circle
                cv2.circle(self.image_output, (i[0], i[1]), 2, (0, 0, 255), 3)
                # save output to variables
                if i[2] > biggest_radius:
                    self.circle_x = round(i[0] / self.camera_width * 2 - 1, 3)
                    self.circle_y = round(i[1] / self.camera_height * 2 - 1, 3)
                    self.circle_z = i[2]
                    biggest_radius = self.circle_z
                    
    def get_circle_x(self):
        return self.circle_x