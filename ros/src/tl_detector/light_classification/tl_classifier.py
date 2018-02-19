from styx_msgs.msg import TrafficLight
import sys
KERAS_RETINANET_PATH="../../../keras-retinanet"
sys.path.append(KERAS_RETINANET_PATH)

import rospy

# import keras
import keras #2.1.3 is the requirement

from keras_retinanet.models.resnet import custom_objects
from keras_retinanet.utils.image import read_image_bgr, preprocess_image, resize_image

# import miscellaneous modules
import cv2
import os
import numpy as np
import time

# set tf backend to allow memory to grow, instead of claiming everything
import tensorflow as tf

#import test
class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        self.model = None

    def init_retinanet(self):
        def get_session():
            config = tf.ConfigProto()
            config.gpu_options.allow_growth = True
            return tf.Session(config=config)

        # set the modified tf session as backend in keras
        keras.backend.tensorflow_backend.set_session(get_session())


        try:
            model_path = os.path.join(KERAS_RETINANET_PATH, 'snapshots', 'd00_9_csv_34.h5')
            self.model = keras.models.load_model(model_path, custom_objects=custom_objects)
        except IOError:
            rospy.logerr("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            rospy.logerr("PLEASE DOWNLOAD the trainted model d00_9_csv_34.h5 file")
            rospy.logerr("                into ./keras-retinanet/snapshots")
            rospy.logerr("                from https://www.dropbox.com/s/xpifcgnw0lcd5ce/d00_9_csv_34.h5?dl=0")
            rospy.logerr("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            sys.exit(-1)

	self.labels_to_names = {
	    0: TrafficLight.RED,
	    1: TrafficLight.YELLOW,
	    2: TrafficLight.GREEN,
	}

	self.labels_to_strs = {
	    TrafficLight.RED: 'RED',
	    TrafficLight.YELLOW: 'YELLOW',
	    TrafficLight.GREEN: 'GREEN',
            TrafficLight.UNKNOWN: "UNKNOWN",
	}


        rospy.loginfo("------------------------------------------------------------------")
        rospy.loginfo("Setup Done")
        rospy.loginfo("------------------------------------------------------------------")

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        if self.model == None:
            self.init_retinanet() # initialization should be done in the same thread

	image = preprocess_image(image)
	image, _ = resize_image(image)

	_, _, detections = self.model.predict(np.expand_dims(image, axis=0))

	# compute predicted labels and scores
	predicted_labels = np.argmax(detections[0, :, 4:], axis=1)
	scores = detections[0, np.arange(detections.shape[1]), 4 + predicted_labels]


	# visualize detections
	labels_scr = np.array([0.,0.,0.])
	labels_cnt = np.array([0.,0.,0.])
	for idx, (label, score) in enumerate(zip(predicted_labels, scores)):
	    if score < 0.1:
		continue
	    labels_cnt[label] += 1
	    labels_scr[label] += score

	weighted_scr = labels_scr / labels_cnt
	weighted_scr = [(0. if np.isnan(e) or np.isinf(e) else e) for e in weighted_scr]
	ix = np.argmax(weighted_scr)

        if ix == 0 and labels_cnt[0] == 0:
            res = TrafficLight.UNKNOWN
        else:
            res = self.labels_to_names[ix]

        rospy.logdebug("traffic light: {}".format(self.labels_to_strs[res]))
        return res
