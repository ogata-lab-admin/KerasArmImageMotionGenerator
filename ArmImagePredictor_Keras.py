#!/usr/bin/env python
# -*- coding: utf-8 -*-
# -*- Python -*-

"""
 @file ArmImagePredictor_Keras.py
 @brief Arm Image Predictor using Keras RT Component
 @date $Date$


"""
import sys
import time
sys.path.append(".")

# Import RTM module
import RTC
import OpenRTM_aist

import ManipulatorCommonInterface_DataTypes_idl
import ManipulatorCommonInterface_Common_idl
import ManipulatorCommonInterface_MiddleLevel_idl
import Img


import numpy as np
import cv2


import os, math
import keras
from keras.utils import np_utils
from keras.layers.convolutional import Conv2D, MaxPooling2D
from keras.models import Sequential, model_from_json
from keras.layers.core import Dense, Dropout, Activation, Flatten
from keras.preprocessing.image import array_to_img, img_to_array, load_img#,list_pictures, load_img
import numpy as np
import pandas as pd
from sklearn.model_selection import train_test_split
# import matplotlib.pyplot as plt

# Import Service implementation class
# <rtc-template block="service_impl">

# </rtc-template>

# Import Service stub modules
# <rtc-template block="consumer_import">
import JARA_ARM, JARA_ARM__POA
import JARA_ARM, JARA_ARM__POA


# </rtc-template>


# This module's spesification
# <rtc-template block="module_spec">
         "type_name",         "ArmImagePredictor_Keras", 
         "description",       "Arm Image Predictor using Keras RT Component", 
         "version",           "1.0.0", 
         "vendor",            "ogata_lab", 
         "category",          "Experimental", 
         "activity_type",     "STATIC", 
         "max_instance",      "1", 
         "language",          "Python", 
         "lang_type",         "SCRIPT",
         "conf.default.debug", "1",
         "conf.default.gripper_close_ratio", "0.1",
                                
         "conf.default.camera_jointPos0", "1.57076",
         "conf.default.camera_jointPos1", "0",
         "conf.default.camera_jointPos2", "1.57076",
         "conf.default.camera_jointPos3", "0",
         "conf.default.camera_jointPos4", "1.57076",
         "conf.default.camera_jointPos5", "0",

         "conf.default.initial_jointPos0", "1.57076",
         "conf.default.initial_jointPos1", "0",
         "conf.default.initial_jointPos2", "1.57076",
         "conf.default.initial_jointPos3", "0",
         "conf.default.initial_jointPos4", "1.57076",
         "conf.default.initial_jointPos5", "0",
                                
         "conf.__widget__.debug", "text",

         "conf.__type__.debug", "int",

         "conf.__widget__.gripper_close_ratio", "text",
                                
         "conf.__widget__.camera_jointPos0", "text",
         "conf.__widget__.camera_jointPos1", "text",
         "conf.__widget__.camera_jointPos2", "text",
         "conf.__widget__.camera_jointPos3", "text",
         "conf.__widget__.camera_jointPos4", "text",
         "conf.__widget__.camera_jointPos5", "text",

                                
         "conf.__widget__.initial_jointPos0", "text",
         "conf.__widget__.initial_jointPos1", "text",
         "conf.__widget__.initial_jointPos2", "text",
         "conf.__widget__.initial_jointPos3", "text",
         "conf.__widget__.initial_jointPos4", "text",
         "conf.__widget__.initial_jointPos5", "text",

                                
         #"conf.__type__.gripper_close_ratio", "float",
                  
         ""]
=======
		 "type_name",         "ArmImagePredictor_Keras", 
		 "description",       "Arm Image Predictor using Keras RT Component", 
		 "version",           "1.0.0", 
		 "vendor",            "ogata_lab", 
		 "category",          "Experimental", 
		 "activity_type",     "STATIC", 
		 "max_instance",      "1", 
		 "language",          "Python", 
		 "lang_type",         "SCRIPT",
		 "conf.default.debug", "1",
         "conf.default.gripper_close_ratio", "0.1",

		 "conf.__widget__.debug", "text",

         "conf.__type__.debug", "int",

         "conf.__widget__.debug", "text",

         "conf.__type__.debug", "int",
                  
		 ""]

# </rtc-template>

##
# @class ArmImagePredictor_Keras
# @brief Arm Image Predictor using Keras RT Component
# 
# 
class ArmImagePredictor_Keras(OpenRTM_aist.DataFlowComponentBase):
    
    ##
    # @brief constructor
    # @param manager Maneger Object
    # 
    def __init__(self, manager):
        OpenRTM_aist.DataFlowComponentBase.__init__(self, manager)

        camera_arg = [None] * ((len(Img._d_TimedCameraImage) - 4) / 2)
        self._d_camera = Img.TimedCameraImage(*camera_arg)
        """
        """
        self._cameraIn = OpenRTM_aist.InPort("camera", self._d_camera)

        """
        """
        self._manipCommonPort = OpenRTM_aist.CorbaPort("manipCommon")
        """
        """
        self._manipMiddlePort = OpenRTM_aist.CorbaPort("manipMiddle")

        

        """
        """
        self._manipCommon = OpenRTM_aist.CorbaConsumer(interfaceType=JARA_ARM.ManipulatorCommonInterface_Common)
        """
        """
        self._manipMiddle = OpenRTM_aist.CorbaConsumer(interfaceType=JARA_ARM.ManipulatorCommonInterface_Middle)

        # initialize of configuration-data.
        # <rtc-template block="init_conf_param">
        """
        
         - Name:  debug
         - DefaultValue: 1
        """
        self._debug = [1]

        self._model = None
        
        """
        
         - Name:  gripper_close_ratio
         - DefaultValue: 0.1
        """
        self._gripper_close_ratio = [0.1]

    #self._model = None

        # the position for taking a picture
        self._camera_jointPos0 = [1.57076]
        self._camera_jointPos1 = [0]
        self._camera_jointPos2 = [1.57076]
        self._camera_jointPos3 = [0]
        self._camera_jointPos4 = [1.57076]
        self._camera_jointPos5 = [0]
        
        #the initial position
        self._initial_jointPos0 = [1.57076]
        self._initial_jointPos1 = [0]
        self._initial_jointPos2 = [1.57076]
        self._initial_jointPos3 = [0]
        self._initial_jointPos4 = [1.57076]
        self._initial_jointPos5 = [0]
        
        # </rtc-template>


         
    ##
    #
    # The initialize action (on CREATED->ALIVE transition)
    # formaer rtc_init_entry() 
    # 
    # @return RTC::ReturnCode_t
    # 
    #
    def onInitialize(self):
        # Bind variables and configuration variable
        self.bindParameter("debug", self._debug, "1")
        self.bindParameter("gripper_close_ratio", self._gripper_close_ratio, "0.1")
        
        self.bindParameter("camera_jointPos0", self._camera_jointPos0, "1.57076")
        self.bindParameter("camera_jointPos1", self._camera_jointPos1, "0")
        self.bindParameter("camera_jointPos2", self._camera_jointPos2, "1.57076")
        self.bindParameter("camera_jointPos3", self._camera_jointPos3, "0")
        self.bindParameter("camera_jointPos4", self._camera_jointPos4, "1.57076")
        self.bindParameter("camera_jointPos5", self._camera_jointPos5, "0")        
        
        
        self.bindParameter("initial_jointPos0", self._initial_jointPos0, "1.57076")
        self.bindParameter("initial_jointPos1", self._initial_jointPos1, "0")
        self.bindParameter("initial_jointPos2", self._initial_jointPos2, "1.57076")
        self.bindParameter("initial_jointPos3", self._initial_jointPos3, "0")
        self.bindParameter("initial_jointPos4", self._initial_jointPos4, "1.57076")
        self.bindParameter("initial_jointPos5", self._initial_jointPos5, "0")        
        
        # Set InPort buffers
        self.addInPort("camera",self._cameraIn)
        
        # Set OutPort buffers
        
        # Set service provider to Ports
        
        # Set service consumers to Ports
        self._manipCommonPort.registerConsumer("JARA_ARM_ManipulatorCommonInterface_Common", "JARA_ARM::ManipulatorCommonInterface_Common", self._manipCommon)
        self._manipMiddlePort.registerConsumer("JARA_ARM_ManipulatorCommonInterface_Middle", "JARA_ARM::ManipulatorCommonInterface_Middle", self._manipMiddle)
        
        # Set CORBA Service Ports
        self.addPort(self._manipCommonPort)
        self.addPort(self._manipMiddlePort)
        
        return RTC.RTC_OK
    
    #   ##
    #   # 
    #   # The finalize action (on ALIVE->END transition)
    #   # formaer rtc_exiting_entry()
    #   # 
    #   # @return RTC::ReturnCode_t
    #
    #   # 
    #def onFinalize(self):
    #
    #   return RTC.RTC_OK
    
    #   ##
    #   #
    #   # The startup action when ExecutionContext startup
    #   # former rtc_starting_entry()
    #   # 
    #   # @param ec_id target ExecutionContext Id
    #   #
    #   # @return RTC::ReturnCode_t
    #   #
    #   #
    #def onStartup(self, ec_id):
    #
    #   return RTC.RTC_OK
    
    #   ##
    #   #
    #   # The shutdown action when ExecutionContext stop
    #   # former rtc_stopping_entry()
    #   #
    #   # @param ec_id target ExecutionContext Id
    #   #
    #   # @return RTC::ReturnCode_t
    #   #
    #   #
    #def onShutdown(self, ec_id):
    #
    #   return RTC.RTC_OK
    
        ##
        #
        # The activated action (Active state entry action)
        # former rtc_active_entry()
        #
        # @param ec_id target ExecutionContext Id
        # 
        # @return RTC::ReturnCode_t
        #
        #
    def onActivated(self, ec_id):
        #c = pd.read_csv(os.path.join(dir, 'joints.csv'))
        #Y = [y for y in zip((c['x']-0.12)/0.12, (c['y']+0.12)/0.24, (c['theta']+math.pi)/(2*math.pi))]
        #X = [img_to_array(load_img(os.path.join(dir, png.strip()), target_size=(64,64)))/256 for png in c['ImageFilename']]
        print ('onActivated')
        self._model = model_from_json(open('model_log.json', 'r').read())
        self._model.compile(loss='mean_squared_error',
                    optimizer='SGD',
                    metrics=['accuracy'])

        self._model.load_weights('param.hdf5')
        print ('OK')


        self._manipCommon._ptr().servoON()
        self._manipMiddle._ptr().setSpeedJoint(30)

        self._manipMiddle._ptr().movePTPJointAbs([self._camera_jointPos0[0], self._camera_jointPos1[0], self._camera_jointPos2[0], self._camera_jointPos3[0], self._camera_jointPos4[0], self._camera_jointPos5[0], self._camera_jointPos6[0]])  #[math.pi/2,0, math.pi/2, 0, math.pi/2, 0]
        self._manipMiddle._ptr().moveGripper(50)

        return RTC.RTC_OK
    
        ##
        #
        # The deactivated action (Active state exit action)
        # former rtc_active_exit()
        #
        # @param ec_id target ExecutionContext Id
        #
        # @return RTC::ReturnCode_t
        #
        #
    def onDeactivated(self, ec_id):
    
        return RTC.RTC_OK
    
        ##
        #
        # The execution action that is invoked periodically
        # former rtc_active_do()
        #
        # @param ec_id target ExecutionContext Id
        #
        # @return RTC::ReturnCode_t
        #
        #
    def onExecute(self, ec_id):

        if self._cameraIn.isNew():
            raw_input('Hello?:')
            data = self._cameraIn.read()
            w = data.data.image.width
            h = data.data.image.height
            print w, h
            img = np.ndarray(shape=(h, w, 3), dtype=float)
            size = len(data.data.image.raw_data)
            for i in range(w*h):
                img[i/w, i%w, 0] = ord(data.data.image.raw_data[i*3+0])
                img[i/w, i%w, 1] = ord(data.data.image.raw_data[i*3+1])
                img[i/w, i%w, 2] = ord(data.data.image.raw_data[i*3+2])
                pass
            # cv2.imwrite('hoge2.png', img)
            #img23 = cv2.cvtColor(img, cv2.COLOR_BGR2RGB).astype(np.float32)
            #cv2.imwrite('hoge23.png', img23)
            img3 = cv2.resize(img, (128, 128))
            # X = [img_to_array(load_img(os.path.join(dir, png.strip()), target_size=(64,64)))/256 for png in c['ImageFilename']]
            result = self._model.predict(np.asarray([img3/256]))
            x = result[0][0]*0.12 + 0.12
            y = result[0][1]*0.24 - 0.12
            th = result[0][2]*2*math.pi-math.pi
            print 'result',
            print x, y, th

            z = 40 / 1000.0
            z_min = 10 / 1000.0

            #JARA_ARM.CarPosWithElbow carPos
            s2 = math.sin(th)
            c2 = math.cos(th)
            carPos = JARA_ARM.CarPosWithElbow([[0,0,0,0],[0,0,0,0],[0,0,0,0]], 1.0, 1)
            carPos.carPos[0][0] = -c2;  carPos.carPos[0][1] = s2; carPos.carPos[0][2] =  0.0; carPos.carPos[0][3] = x;
            carPos.carPos[1][0] =  s2;  carPos.carPos[1][1] = c2; carPos.carPos[1][2] =  0.0; carPos.carPos[1][3] = y;
            carPos.carPos[2][0] =  0.0; carPos.carPos[2][1] = 0; carPos.carPos[2][2] = -1.0; carPos.carPos[2][3] = z;
            self._manipMiddle._ptr().movePTPCartesianAbs(carPos)
            
            time.sleep(1.0)

            carPos.carPos[2][3] = z_min
            self._manipMiddle._ptr().movePTPCartesianAbs(carPos)
            time.sleep(1.0)

            m_gripper_close_ratio = 0.0
            
            if self._gripper_close_ratio > 1.0:
                m_gripper_close_ratio = 1.0
            elif self._gripper_close_ratio < 0.0:
                m_gripper_close_ratio = 0.0
            else:
                m_gripper_close_ratio = self._gripper_close_ratio[0] * 100
                
            self._manipMiddle._ptr().moveGripper(int(m_gripper_close_ratio))
            time.sleep(1.0)

            carPos.carPos[2][3] = z
            self._manipMiddle._ptr().movePTPCartesianAbs(carPos)
            time.sleep(1.0)
            
            self._manipMiddle._ptr().movePTPJointAbs([self._initial_jointPos0[0], self._initial_jointPos1[0], self._initial_jointPos2[0], self._initial_jointPos3[0], self._initial_jointPos4[0], self._initial_jointPos5[0], self._initial_jointPos6[0]])  #[math.pi/2,0, math.pi/2, 0, math.pi/2, 0]         
            time.sleep(3.0)
            
            self._manipMiddle._ptr().moveGripper(50)

            if self._camera_jointPos0 == self._initial_jointPos0: #|| self._camera_jointPos1 == self._initial_jointPos1 || self._camera_jointPos2 == self._initial_jointPos2 || self._camera_jointPos3 == self._initial_jointPos3 || self._camera_jointPos4 == self._initial_jointPos4 || self._camera_jointPos5 == self._initial_jointPos5:
                self._manipMiddle._ptr().movePTPJointAbs([self._camera_jointPos0[0], self._camera_jointPos1[0], self._camera_jointPos2[0], self._camera_jointPos3[0], self._camera_jointPos4[0], self._camera_jointPos5[0], self._camera_jointPos6[0]])


        return RTC.RTC_OK
    
    #   ##
    #   #
    #   # The aborting action when main logic error occurred.
    #   # former rtc_aborting_entry()
    #   #
    #   # @param ec_id target ExecutionContext Id
    #   #
    #   # @return RTC::ReturnCode_t
    #   #
    #   #
    #def onAborting(self, ec_id):
    #
    #   return RTC.RTC_OK
    
    #   ##
    #   #
    #   # The error action in ERROR state
    #   # former rtc_error_do()
    #   #
    #   # @param ec_id target ExecutionContext Id
    #   #
    #   # @return RTC::ReturnCode_t
    #   #
    #   #
    #def onError(self, ec_id):
    #
    #   return RTC.RTC_OK
    
    #   ##
    #   #
    #   # The reset action that is invoked resetting
    #   # This is same but different the former rtc_init_entry()
    #   #
    #   # @param ec_id target ExecutionContext Id
    #   #
    #   # @return RTC::ReturnCode_t
    #   #
    #   #
    #def onReset(self, ec_id):
    #
    #   return RTC.RTC_OK
    
    #   ##
    #   #
    #   # The state update action that is invoked after onExecute() action
    #   # no corresponding operation exists in OpenRTm-aist-0.2.0
    #   #
    #   # @param ec_id target ExecutionContext Id
    #   #
    #   # @return RTC::ReturnCode_t
    #   #

    #   #
    #def onStateUpdate(self, ec_id):
    #
    #   return RTC.RTC_OK
    
    #   ##
    #   #
    #   # The action that is invoked when execution context's rate is changed
    #   # no corresponding operation exists in OpenRTm-aist-0.2.0
    #   #
    #   # @param ec_id target ExecutionContext Id
    #   #
    #   # @return RTC::ReturnCode_t
    #   #
    #   #
    #def onRateChanged(self, ec_id):
    #
    #   return RTC.RTC_OK
    
	
	##
	# @brief constructor
	# @param manager Maneger Object
	# 
	def __init__(self, manager):
		OpenRTM_aist.DataFlowComponentBase.__init__(self, manager)

		camera_arg = [None] * ((len(Img._d_TimedCameraImage) - 4) / 2)
		self._d_camera = Img.TimedCameraImage(*camera_arg)
		"""
		"""
		self._cameraIn = OpenRTM_aist.InPort("camera", self._d_camera)

		"""
		"""
		self._manipCommonPort = OpenRTM_aist.CorbaPort("manipCommon")
		"""
		"""
		self._manipMiddlePort = OpenRTM_aist.CorbaPort("manipMiddle")

		

		"""
		"""
		self._manipCommon = OpenRTM_aist.CorbaConsumer(interfaceType=JARA_ARM.ManipulatorCommonInterface_Common)
		"""
		"""
		self._manipMiddle = OpenRTM_aist.CorbaConsumer(interfaceType=JARA_ARM.ManipulatorCommonInterface_Middle)

		# initialize of configuration-data.
		# <rtc-template block="init_conf_param">
		"""
		
		 - Name:  debug
		 - DefaultValue: 1
		"""
		self._debug = [1]

		self._model = None
        
		"""
		
		 - Name:  gripper_close_ratio
		 - DefaultValue: 0.1
		"""
		self._gripper_close_ratio = [0.1]

		self._model = None
				
		# </rtc-template>


		 
	##
	#
	# The initialize action (on CREATED->ALIVE transition)
	# formaer rtc_init_entry() 
	# 
	# @return RTC::ReturnCode_t
	# 
	#
	def onInitialize(self):
		# Bind variables and configuration variable
		self.bindParameter("debug", self._debug, "1")
		self.bindParameter("gripper_close_ratio", self._gripper_close_ratio, "0.1")
        
		# Set InPort buffers
		self.addInPort("camera",self._cameraIn)
		
		# Set OutPort buffers
		
		# Set service provider to Ports
		
		# Set service consumers to Ports
		self._manipCommonPort.registerConsumer("JARA_ARM_ManipulatorCommonInterface_Common", "JARA_ARM::ManipulatorCommonInterface_Common", self._manipCommon)
		self._manipMiddlePort.registerConsumer("JARA_ARM_ManipulatorCommonInterface_Middle", "JARA_ARM::ManipulatorCommonInterface_Middle", self._manipMiddle)
		
		# Set CORBA Service Ports
		self.addPort(self._manipCommonPort)
		self.addPort(self._manipMiddlePort)
		
		return RTC.RTC_OK
	
	#	##
	#	# 
	#	# The finalize action (on ALIVE->END transition)
	#	# formaer rtc_exiting_entry()
	#	# 
	#	# @return RTC::ReturnCode_t
	#
	#	# 
	#def onFinalize(self):
	#
	#	return RTC.RTC_OK
	
	#	##
	#	#
	#	# The startup action when ExecutionContext startup
	#	# former rtc_starting_entry()
	#	# 
	#	# @param ec_id target ExecutionContext Id
	#	#
	#	# @return RTC::ReturnCode_t
	#	#
	#	#
	#def onStartup(self, ec_id):
	#
	#	return RTC.RTC_OK
	
	#	##
	#	#
	#	# The shutdown action when ExecutionContext stop
	#	# former rtc_stopping_entry()
	#	#
	#	# @param ec_id target ExecutionContext Id
	#	#
	#	# @return RTC::ReturnCode_t
	#	#
	#	#
	#def onShutdown(self, ec_id):
	#
	#	return RTC.RTC_OK
	
		##
		#
		# The activated action (Active state entry action)
		# former rtc_active_entry()
		#
		# @param ec_id target ExecutionContext Id
		# 
		# @return RTC::ReturnCode_t
		#
		#
	def onActivated(self, ec_id):
		#c = pd.read_csv(os.path.join(dir, 'joints.csv'))
		#Y = [y for y in zip((c['x']-0.12)/0.12, (c['y']+0.12)/0.24, (c['theta']+math.pi)/(2*math.pi))]
		#X = [img_to_array(load_img(os.path.join(dir, png.strip()), target_size=(64,64)))/256 for png in c['ImageFilename']]
		print ('onActivated')
		self._model = model_from_json(open('model_log.json', 'r').read())
		self._model.compile(loss='mean_squared_error',
				    optimizer='SGD',
				    metrics=['accuracy'])

		self._model.load_weights('param.hdf5')
		print ('OK')


		self._manipCommon._ptr().servoON()
		self._manipMiddle._ptr().setSpeedJoint(30)

		self._manipMiddle._ptr().movePTPJointAbs([math.pi/2,0, math.pi/2, 0, math.pi/2, 0])
		self._manipMiddle._ptr().moveGripper(50)

		return RTC.RTC_OK
	
		##
		#
		# The deactivated action (Active state exit action)
		# former rtc_active_exit()
		#
		# @param ec_id target ExecutionContext Id
		#
		# @return RTC::ReturnCode_t
		#
		#
	def onDeactivated(self, ec_id):
	
		return RTC.RTC_OK
	
		##
		#
		# The execution action that is invoked periodically
		# former rtc_active_do()
		#
		# @param ec_id target ExecutionContext Id
		#
		# @return RTC::ReturnCode_t
		#
		#
	def onExecute(self, ec_id):

		if self._cameraIn.isNew():
			raw_input('Hello?:')
			data = self._cameraIn.read()
			w = data.data.image.width
			h = data.data.image.height
			print w, h
			img = np.ndarray(shape=(h, w, 3), dtype=float)
			size = len(data.data.image.raw_data)
			for i in range(w*h):
				img[i/w, i%w, 0] = ord(data.data.image.raw_data[i*3+0])
				img[i/w, i%w, 1] = ord(data.data.image.raw_data[i*3+1])
				img[i/w, i%w, 2] = ord(data.data.image.raw_data[i*3+2])
				pass
			# cv2.imwrite('hoge2.png', img)
			#img23 = cv2.cvtColor(img, cv2.COLOR_BGR2RGB).astype(np.float32)
			#cv2.imwrite('hoge23.png', img23)
			img3 = cv2.resize(img, (64, 64))
			# X = [img_to_array(load_img(os.path.join(dir, png.strip()), target_size=(64,64)))/256 for png in c['ImageFilename']]
			result = self._model.predict(np.asarray([img3/256]))
			x = result[0][0]*0.12 + 0.12
			y = result[0][1]*0.24 - 0.12
			th = result[0][2]*2*math.pi-math.pi
			print 'result',
			print x, y, th

			z = 40 / 1000.0
			z_min = 10 / 1000.0

			#JARA_ARM.CarPosWithElbow carPos
			s2 = math.sin(th)
			c2 = math.cos(th)
			carPos = JARA_ARM.CarPosWithElbow([[0,0,0,0],[0,0,0,0],[0,0,0,0]], 1.0, 1)
			carPos.carPos[0][0] = -c2;  carPos.carPos[0][1] = s2; carPos.carPos[0][2] =  0.0; carPos.carPos[0][3] = x;
			carPos.carPos[1][0] =  s2;  carPos.carPos[1][1] = c2; carPos.carPos[1][2] =  0.0; carPos.carPos[1][3] = y;
			carPos.carPos[2][0] =  0.0; carPos.carPos[2][1] = 0; carPos.carPos[2][2] = -1.0; carPos.carPos[2][3] = z;
			self._manipMiddle._ptr().movePTPCartesianAbs(carPos)
			
			time.sleep(1.0)

			carPos.carPos[2][3] = z_min
			self._manipMiddle._ptr().movePTPCartesianAbs(carPos)
			time.sleep(1.0)


			if self._gripper_close_ratio > 1.0:
            			m_gripper_close_ratio = 1.0
			elif self._gripper_close_ratio < 0.0:
            			m_gripper_close_ratio = 0.0
           		else:
            			m_gripper_close_ratio = self._gripper_close_ratio
                
            			self._manipMiddle._ptr().moveGripper(m_gripper_close_ratio*100)
                		time.sleep(1.0)

			carPos.carPos[2][3] = z
			self._manipMiddle._ptr().movePTPCartesianAbs(carPos)
			time.sleep(1.0)
			
			self._manipMiddle._ptr().movePTPJointAbs([math.pi/2,0, math.pi/2, 0, math.pi/2, 0])			
			time.sleep(3.0)
			self._manipMiddle._ptr().moveGripper(50)


		return RTC.RTC_OK
	
	#	##
	#	#
	#	# The aborting action when main logic error occurred.
	#	# former rtc_aborting_entry()
	#	#
	#	# @param ec_id target ExecutionContext Id
	#	#
	#	# @return RTC::ReturnCode_t
	#	#
	#	#
	#def onAborting(self, ec_id):
	#
	#	return RTC.RTC_OK
	
	#	##
	#	#
	#	# The error action in ERROR state
	#	# former rtc_error_do()
	#	#
	#	# @param ec_id target ExecutionContext Id
	#	#
	#	# @return RTC::ReturnCode_t
	#	#
	#	#
	#def onError(self, ec_id):
	#
	#	return RTC.RTC_OK
	
	#	##
	#	#
	#	# The reset action that is invoked resetting
	#	# This is same but different the former rtc_init_entry()
	#	#
	#	# @param ec_id target ExecutionContext Id
	#	#
	#	# @return RTC::ReturnCode_t
	#	#
	#	#
	#def onReset(self, ec_id):
	#
	#	return RTC.RTC_OK
	
	#	##
	#	#
	#	# The state update action that is invoked after onExecute() action
	#	# no corresponding operation exists in OpenRTm-aist-0.2.0
	#	#
	#	# @param ec_id target ExecutionContext Id
	#	#
	#	# @return RTC::ReturnCode_t
	#	#

	#	#
	#def onStateUpdate(self, ec_id):
	#
	#	return RTC.RTC_OK
	
	#	##
	#	#
	#	# The action that is invoked when execution context's rate is changed
	#	# no corresponding operation exists in OpenRTm-aist-0.2.0
	#	#
	#	# @param ec_id target ExecutionContext Id
	#	#
	#	# @return RTC::ReturnCode_t
	#	#
	#	#
	#def onRateChanged(self, ec_id):
	#
	#	return RTC.RTC_OK




def ArmImagePredictor_KerasInit(manager):
    profile = OpenRTM_aist.Properties(defaults_str=armimagepredictor_keras_spec)
    manager.registerFactory(profile,
                            ArmImagePredictor_Keras,
                            OpenRTM_aist.Delete)

def MyModuleInit(manager):
    ArmImagePredictor_KerasInit(manager)

    # Create a component
    comp = manager.createComponent("ArmImagePredictor_Keras")

def main():
    mgr = OpenRTM_aist.Manager.init(sys.argv)
    mgr.setModuleInitProc(MyModuleInit)
    mgr.activateManager()
    mgr.runManager()

if __name__ == "__main__":
    main()

