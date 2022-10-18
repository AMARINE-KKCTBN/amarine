from time import sleep
import cv2
import numpy as np
import json
import sys
import os.path
from os import path
import random as rng

hsv_empty = {
    "H Lower" : 0,
    "H Higher" : 0,
    "S Lower" : 0,
    "S Higher" : 0,
    "V Lower" : 0,
    "V Higher" : 0,
}
circle_empty = {
    "min_radius" : 1,
    "max_radius" : 1,
    "param1" : 1,
    "param2" : 1,
    "param3" : 1,
    "param4" : 1,
}
transform_empty = {
    "object_canny_param1" : 50,
    "object_canny_param2" : 150,
    "object_morph_kernel" : 7,
    "field_canny_param1" : 50,
    "field_canny_param2" : 150,
    "field_morph_kernel" : 7,
}

object_hsv_path = 'object_hsv.json'
field_hsv_path = 'field_hsv.json'
circle_params_path = 'circle_params.json'
transform_params_path = 'transform_params.json'


class hsv_detector:
    def read_params(self):
        if path.exists(object_hsv_path):
            with open(object_hsv_path, 'r') as openfile:
                self.object_hsv = json.load(openfile)
                # print("object_hsv exist")
        else:
            self.object_hsv = hsv_empty
            # print("object_hsv does not exist")
            # print(object_hsv)

        if path.exists(field_hsv_path):
            with open(field_hsv_path, 'r') as openfile:
                self.field_hsv = json.load(openfile)
        else:
            self.field_hsv = hsv_empty

        if path.exists(circle_params_path):
            with open(circle_params_path, 'r') as openfile:
                self.circle_params = json.load(openfile)
        else:
            self.circle_params = circle_empty

        if path.exists(transform_params_path):
            with open(transform_params_path, 'r') as openfile:
                self.transform_params = json.load(openfile)
        else:
            self.transform_params = transform_empty
    
    def morph_image(self, img):
        self.object_kernel = np.ones((self.transform_params['object_morph_kernel'],self.transform_params['object_morph_kernel']),"uint8")

        #morphological operations
        self.object_mask = cv2.morphologyEx(img,cv2.MORPH_OPEN, self.object_kernel)
        self.object_mask = cv2.morphologyEx(img,cv2.MORPH_CLOSE, self.object_kernel)

    def 