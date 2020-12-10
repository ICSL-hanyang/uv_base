#!/usr/bin/env python3
# -*- coding: utf-8 -*-

###############################################################################
#

import const

class Config:
    def __init__(self): # model_name):
        self.version = (const.VERSION_MAJOR, const.VERSION_MINOR) 
        self.valid_rate = const.VALID_RATE
        self.fname_ext = const.IMAGE_EXT
        self.data_ext = const.DATA_EXT
        self.num_epochs = const.NUM_EPOCH
        self.batch_size = const.BATCH_SIZE
        self.num_outputs = const.NUM_OUTPUT   # steering_angle, throttle
        self.raw_scale = const.RAW_SCALE      # Multiply raw input by this scale
        self.jitter_tolerance = const.JITTER_TOLERANCE # joystick jitter
       
        self.net_model_type = const.NET_TYPE_NVIDIA 

        self.image_size = (const.IMAGE_WIDTH, const.IMAGE_HEIGHT, const.IMAGE_DEPTH)
        self.capture_area = (const.CROP_X1, const.CROP_Y1, const.CROP_X2, const.CROP_Y2)