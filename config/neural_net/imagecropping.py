# -*- coding: utf-8 -*-
"""
Created on Wed May 23 13:55:21 2018

@author: mir-lab
"""

from PIL import Image
import os.path, sys
from config import Config

path = "/home/mir-lab/Desktop/2018-02-12-17-10-41"
dirs = os.listdir(path)

#Before using this code, remove the .csv or .txt files from that folder

def crop():
    config = Config()
    for item in dirs:
        fullpath = os.path.join(path,item)         #corrected
        if os.path.isfile(fullpath):
            im = Image.open(fullpath)
            f, e = os.path.splitext(fullpath)
            imCrop = im.crop((config.capture_area[0], config.capture_area[1],
                              config.capture_area[2], config.capture_area[3])) #corrected
            imCrop.save(f + '.jpg')

crop()
