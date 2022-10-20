from time import sleep
import cv2
import numpy as np
import json
import sys
import os.path
from os import path
import random as rng




class hsv_detector:
    def __init__(self):
        self.hsv_empty = {
            "H Lower" : 0,
            "H Higher" : 0,
            "S Lower" : 0,
            "S Higher" : 0,
            "V Lower" : 0,
            "V Higher" : 0,
        }
        self.circle_empty = {
            "min_radius" : 1,
            "max_radius" : 1,
            "param1" : 1,
            "param2" : 1,
            "param3" : 1,
            "param4" : 1,
        }
        self.transform_empty = {
            "object_canny_param1" : 50,
            "object_canny_param2" : 150,
            "object_morph_kernel" : 7,
            "field_canny_param1" : 50,
            "field_canny_param2" : 150,
            "field_morph_kernel" : 7,
        }

        self.object_hsv_path = 'object_hsv.json'
        self.field_hsv_path = 'field_hsv.json'
        self.circle_params_path = 'circle_params.json'
        self.transform_params_path = 'transform_params.json'

    def read_params(self):
        if path.exists(self.object_hsv_path):
            with open(self.object_hsv_path, 'r') as openfile:
                self.object_hsv = json.load(openfile)
                # print("object_hsv exist")
        else:
            self.object_hsv = self.hsv_empty
            # print("object_hsv does not exist")
            # print(object_hsv)

        if path.exists(self.field_hsv_path):
            with open(self.field_hsv_path, 'r') as openfile:
                self.field_hsv = json.load(openfile)
        else:
            self.field_hsv = self.hsv_empty

        if path.exists(self.circle_params_path):
            with open(self.circle_params_path, 'r') as openfile:
                self.circle_params = json.load(openfile)
        else:
            self.circle_params = self.circle_empty

        if path.exists(self.transform_params_path):
            with open(self.transform_params_path, 'r') as openfile:
                self.transform_params = json.load(openfile)
        else:
        
            self.transform_params = self.transform_empty
    
    def morph_image(self, img):
        object_kernel = np.ones((self.transform_params['object_morph_kernel'],self.transform_params['object_morph_kernel']),"uint8")

        #morphological operations
        output_img = cv2.morphologyEx(img,cv2.MORPH_OPEN, object_kernel)
        output_img = cv2.morphologyEx(img,cv2.MORPH_CLOSE, object_kernel)
        
        #sub: dilate-erode
        # self.object_mask = cv2.dilate(self.object_mask, self.object_kernel,iterations=1)
        # self.object_mask = cv2.erode(self.object_mask, self.object_kernel, iterations=2)

        return output_img

    def filter_object(self, img):
        #object hsv filter params
        object_LowerRegion = np.array([self.object_hsv['H Lower'],self.object_hsv['S Lower'],self.object_hsv['V Lower']],np.uint8)
        object_upperRegion = np.array([self.object_hsv['H Higher'],self.object_hsv['S Higher'],self.object_hsv['V Higher']],np.uint8)
        
        self.object_mask = cv2.inRange(self.object_mask, object_LowerRegion, object_upperRegion)