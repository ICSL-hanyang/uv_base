#!/usr/bin/env python3
# -*- coding: utf-8 -*-
'''
Created on Fri May 17 3:13:00 2019

@author: ninad
'''

import numpy as np
import sys
import os
import pandas as pd
from progressbar import ProgressBar
from scipy import misc
from drive_run import DriveRun
from config import Config
from image_process import ImageProcess

#sys.path.append('/home/ghor9797/NCD_Guthub/python/keras-vis')
#sys.path.append(str(os.environ['HOME']) + ('/keras-vis'))
from vis.utils import utils
from vis.visualization import visualize_saliency, overlay

def main():

    if (len(sys.argv) != 2):
        print('Give the model path.')
        return

    drive = DriveRun(sys.argv[1])
    config = Config()
    csv_fname = '/home/ghor9797/NCD_Github/test/test.csv'
    csv_header = ['image_fname', 'steering_angle']
    df = pd.read_csv(csv_fname, names=csv_header, index_col=False)
    num_data = len(df)
    text = open('/home/ghor9797/NCD_Github/test/salient.txt', 'w+')
    bar = ProgressBar()
    image_process = ImageProcess()

    for i in bar(range(num_data)):
        image_name = df.loc[i]['image_fname']
        steering = df.loc[i]['steering_angle']
        image_path = '/home/ghor9797/NCD_Github/test/' + image_name + '.jpg'
        image = utils.load_img(image_path, target_size=(config.image_size[1],
                                                        config.image_size[0]))
        image = image_process.process(image)
        prediction = drive.run(image)
        text.write(str(image_name) + '\t' + str(steering) + '\t' + str(prediction))
        
        modifiers = [None, 'negate', 'small_values']
        for i, modifier in enumerate(modifiers):
            heatmap = visualize_saliency(drive.net_model.model, layer_idx=-1,
                                         filter_indices=0, seed_input=image,
                                         grad_modifier=modifier, keepdims=True)
            final = overlay(image, heatmap, alpha=0.5)
            cv2.imwrite('/home/ghor9797/NCD_Github/test/' + image_name + '_' + str(i) + '.jpg', final)
