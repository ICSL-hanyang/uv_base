#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue April 11 14:35 2019

@author: ninad
"""

from keras.models import Sequential, Model
from keras.layers import Lambda, Dropout, Flatten, Dense, Activation, Concatenate
from keras.layers import Conv2D, MaxPooling2D, BatchNormalization, Input
from keras import losses, optimizers

from config import Config

class NetModel:
    def __init__(self, model_path):
        self.model = None
        model_name = model_path[model_path.rfind('/'):] # get folder name
        self.name = model_name.strip('/')

        self.model_path = model_path
        self.config = Config()

        if self.config.typeofModel == 3:
            self.base_model = None
            self.x = None
            self.prediction = None

        if self.config.typeofModel == 6:
            self.input = None
            self.Lambda = None
            self.conv1 = None
            self.batch1 = None
            self.activ1 = None
            self.maxpool1 = None
            self.fire2_squeeze = None
            self.fire2_expand1 = None
            self.fire2_expand2 = None
            self.merge2 = None
            self.maxpool2 = None
            self.fire3_squeeze = None
            self.fire3_expand1 = None
            self.fire3_expand2 = None
            self.merge3 = None
            self.maxpool3 = None
            self.fire4_squeeze = None
            self.fire4_expand1 = None
            self.fire4_expand2 = None
            self.merge4 = None
            self.maxpool4 = None
            self.fire5_squeeze = None
            self.fire5_expand1 = None
            self.fire5_expand2 = None
            self.merge5 = None
            self.conv6 = None
            self.batch6 = None
            self.activ6 = None
            self.maxpool6 = None
            self.flat = None
            self.dense7 = None
            self.batch7 = None
            self.activ7 = None
            self.dropout7 = None
            self.dense8 = None
            self.batch8 = None
            self.activ8 = None
            self.dense9 = None

        self._model()
        
    ###########################################################################
    #
    def _model(self):
        
        if self.config.typeofModel == 1:
        
            input_shape = (self.config.image_size[1], self.config.image_size[0],
                           self.config.image_size[2])

            self.model = Sequential([
                          Lambda(lambda x: x/127.5 - 1.0, input_shape=input_shape),
                          Conv2D(24, (5, 5), activation='relu', padding='same'),
                          MaxPooling2D(pool_size=(2, 2), strides=(1, 1)),
                          Conv2D(36, (5, 5), activation='relu', padding='same'),
                          MaxPooling2D(pool_size=(2, 2)),
                          Conv2D(48, (5, 5), activation='relu', padding='same'),
                          MaxPooling2D(pool_size=(2, 2)),
                          Conv2D(64, (5, 5), activation='relu', padding='same'),
                          MaxPooling2D(pool_size=(2, 2)),
                          Conv2D(64, (5, 5), activation='relu', padding='same'),
                          MaxPooling2D(pool_size=(2, 2)),
                          Flatten(),
                          Dropout(0.5),
                          Dense(256, activation='relu'),
                          Dropout(0.5),
                          Dense(128, activation='relu'),
                          Dropout(0.5),
                          Dense(64, activation='relu'),
                          Dense(self.config.num_outputs)])
            self.model.summary()
            self._compile()

        ###----------------------------------------------------------------###

        elif self.config.typeofModel == 2:

            input_shape = (self.config.image_size[1], self.config.image_size[0],
                           self.config.image_size[2])

            self.model = Sequential([
                          Lambda(lambda x: x/127.5 - 1.0, input_shape=input_shape),
                          Conv2D(24, (5, 5), strides=(2,2), activation='relu'),
                          Conv2D(36, (5, 5), strides=(2,2), activation='relu'),
                          Conv2D(48, (5, 5), strides=(2,2), activation='relu'),
                          Conv2D(64, (3, 3), activation='relu'),
                          Conv2D(64, (3, 3), activation='relu'),
                          Flatten(),
                          Dense(100, activation='relu'),
                          Dense(50, activation='relu'),
                          Dense(10, activation='relu'),
                          Dense(self.config.num_outputs)])
            self.model.summary()
            self._compile()

        ###----------------------------------------------------------------###

        elif self.config.typeofModel == 3:
            
            from keras.applications.resnet50 import ResNet50
        
            input_shape = (self.config.image_size[1], self.config.image_size[0],
                           self.config.image_size[2])
        
            self.base_model = ResNet50(weights='imagenet', include_top=False, input_shape=input_shape)
            self.x = self.base_model.get_layer('activation_10').output
            #self.x = self.base_model.output
            self.x = Conv2D(128, (5, 5), activation='relu')(self.x)
            #self.x = BatchNormalization()(self.x)
            #self.x = Activation('relu')(self.x)
            self.x = MaxPooling2D(pool_size=(2, 2))(self.x) 
            self.x = Conv2D(64, (5, 5), activation='relu')(self.x)
            #self.x = BatchNormalization()(self.x)
            #self.x = Activation('relu')(self.x)
            self.x = MaxPooling2D(pool_size=(2, 2))(self.x)
            self.x = Conv2D(48, (5, 5), activation='relu')(self.x)
            #self.x = BatchNormalization()(self.x)
            #self.x = Activation('relu')(self.x)
            self.x = MaxPooling2D(pool_size=(2, 2))(self.x)  
            self.x = Flatten()(self.x)
            self.x = Dense(256)(self.x)
            self.x = BatchNormalization()(self.x)
            self.x = Activation('relu')(self.x)
            self.x = Dense(128)(self.x)
            self.x = BatchNormalization()(self.x)
            self.x = Activation('relu')(self.x)
            self.x = Dense(64)(self.x)
            self.x = BatchNormalization()(self.x)
            self.x = Activation('relu')(self.x)
            self.prediction = Dense(self.config.num_outputs)(self.x)
            self.model = Model(inputs=self.base_model.input, outputs=self.prediction)
            
            self.model.summary()
            self._compile()

        ###----------------------------------------------------------------###

        elif self.config.typeofModel == 4:

            from keras.layers.recurrent import LSTM
            from keras.layers.wrappers import TimeDistributed
        
            input_shape = (None, self.config.image_size[1], self.config.image_size[0],
                           self.config.image_size[2])
 
            self.model = Sequential([
                          TimeDistributed(Lambda(lambda x: x/127.5 - 1.0), input_shape=input_shape),
                          TimeDistributed(Conv2D(24, (5, 5), strides=(2,2), activation='relu')),
                          TimeDistributed(Conv2D(36, (5, 5), strides=(2,2), activation='relu')),
                          TimeDistributed(Conv2D(48, (5, 5), strides=(2,2), activation='relu')),
                          TimeDistributed(Conv2D(64, (3, 3), activation='relu')),
                          TimeDistributed(Conv2D(64, (3, 3), activation='relu')),
                          TimeDistributed(Flatten()),
                          Dense(100),
                          LSTM(return_sequences=True, units=10),
                          Dense(50),
                          Dense(10),
                          Dropout(0.25),
                          Dense(self.config.num_outputs)])
            self.model.summary()
            self._compile()

        ###----------------------------------------------------------------###

        elif self.config.typeofModel == 5:

            from keras.layers.recurrent import LSTM
            from keras.layers.wrappers import TimeDistributed
        
            input_shape = (None, self.config.image_size[1], self.config.image_size[0],
                           self.config.image_size[2])
 
            self.model = Sequential([
                          TimeDistributed(Lambda(lambda x: x/127.5 - 1.0), input_shape=input_shape),
                          TimeDistributed(Conv2D(24, (5, 5), strides=(2,2), activation='relu')),
                          TimeDistributed(Conv2D(36, (5, 5), strides=(2,2), activation='relu')),
                          TimeDistributed(Conv2D(48, (5, 5), strides=(2,2), activation='relu')),
                          TimeDistributed(Conv2D(64, (3, 3), activation='relu')),
                          TimeDistributed(Conv2D(64, (3, 3), activation='relu')),
                          Dropout(0.25),
                          TimeDistributed(Flatten()),
                          TimeDistributed(Dense(100, activation='relu')),
                          TimeDistributed(Dense(50, activation='relu')),
                          TimeDistributed(Dense(10, activation='relu')),
                          LSTM(return_sequences=True, units=1),
                          Dense(self.config.num_outputs)])
            self.model.summary()
            self._compile()

        ###----------------------------------------------------------------###

        elif self.config.typeofModel == 6:

            self.input = Input(shape=(self.config.image_size[1], self.config.image_size[0],
                                      self.config.image_size[2]))

            #input_shape = (self.config.image_size[1], self.config.image_size[0],
            #              self.config.image_size[2])

            #self.Lambda = Lambda(lambda x: x/127.5 - 1.0, input_shape=input_shape)
            self.Lambda = Lambda(lambda x: x/127.5 - 1.0)(self.input)
            self.conv1 = Conv2D(96, (7,7), padding='same')(self.Lambda)
            self.batch1 = BatchNormalization()(self.conv1)
            self.activ1 = Activation('relu')(self.batch1)
            self.maxpool1 = MaxPooling2D(pool_size=(2,2))(self.activ1) #(3,3)

            self.fire2_squeeze = Conv2D(24, (1,1), activation='relu', padding='same')(self.maxpool1)
            self.fire2_expand1 = Conv2D(36, (1,1), activation='relu', padding='same')(self.fire2_squeeze)
            self.fire2_expand2 = Conv2D(36, (3,3), activation='relu', padding='same')(self.fire2_squeeze)
            self.merge2 = Concatenate()([self.fire2_expand1, self.fire2_expand2])
            self.maxpool2 = MaxPooling2D(pool_size=(2,2))(self.merge2) #(3,3)

            self.fire3_squeeze = Conv2D(36, (1,1), activation='relu', padding='same')(self.maxpool2)
            self.fire3_expand1 = Conv2D(48, (1,1), activation='relu', padding='same')(self.fire3_squeeze)
            self.fire3_expand2 = Conv2D(48, (3,3), activation='relu', padding='same')(self.fire3_squeeze)
            self.merge3 = Concatenate()([self.fire3_expand1, self.fire3_expand2])
            self.maxpool3 = MaxPooling2D(pool_size=(2,2))(self.merge3) #(3,3)

            self.fire4_squeeze = Conv2D(48, (1,1), activation='relu', padding='same')(self.maxpool3)
            self.fire4_expand1 = Conv2D(64, (1,1), activation='relu', padding='same')(self.fire4_squeeze)
            self.fire4_expand2 = Conv2D(64, (3,3), activation='relu', padding='same')(self.fire4_squeeze)
            self.merge4 = Concatenate()([self.fire4_expand1, self.fire4_expand2])
            self.maxpool4 = MaxPooling2D(pool_size=(2,2))(self.merge4) #(3,3)

            #self.fire5_squeeze = Conv2D(64, (1,1), activation='relu', padding='same')(self.maxpool4)
            #self.fire5_expand1 = Conv2D(128, (1,1), activation='relu', padding='same')(self.fire5_squeeze)
            #self.fire5_expand2 = Conv2D(128, (3,3), activation='relu', padding='same')(self.fire5_squeeze)
            #self.merge5 = Concatenate()([self.fire5_expand1, self.fire5_expand2])
            
            self.conv6 = Conv2D(128, (5,5), padding='same')(self.merge4)
            self.batch6 = BatchNormalization()(self.conv6)
            self.activ6 = Activation('relu')(self.batch6)
            self.maxpool6 = MaxPooling2D(pool_size=(2,2))(self.batch6)            

            self.flat = Flatten()(self.maxpool6)

            self.dense7 = Dense(128)(self.flat)
            self.batch7 = BatchNormalization()(self.dense7)
            self.activ7 = Activation('relu')(self.batch7)
            self.dropout7 = Dropout(0.5)(self.activ7)

            self.dense8 = Dense(64)(self.dropout7)
            self.batch8 = BatchNormalization()(self.dense8)
            self.activ8 = Activation('relu')(self.batch8)

            self.dense9 = Dense(self.config.num_outputs)(self.activ8)

            self.model = Model(inputs=self.input, outputs=self.dense9)
            self.model.summary()
            self._compile()

        ###----------------------------------------------------------------###

        else:
            print('Select proper type of Model')
            return


        
        
    ##########################################################################
    #
    def _compile(self):
        self.model.compile(loss=losses.mean_squared_error,
                  optimizer=optimizers.Adam())


    ###########################################################################
    #
    # save model
    def save(self):
        
        json_string = self.model.to_json()
        open(self.model_path+'.json', 'w').write(json_string)
        self.model.save_weights(self.model_path+'.h5', overwrite=True)
    
    
    ###########################################################################
    # model_path = '../data/2007-09-22-12-12-12.
    def load(self):
        
        from keras.models import model_from_json
        
        self.model = model_from_json(open(self.model_path+'.json').read())
        self.model.load_weights(self.model_path+'.h5')
        self._compile()
        
    ###########################################################################
    #
    # show summary
    def summary(self):
        self.model.summary()
        
