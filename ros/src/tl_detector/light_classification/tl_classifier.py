from styx_msgs.msg import TrafficLight

import os
import numpy as np
import tensorflow as tf
import sys

import rospy
from utils import label_utils  
from utils import visual_utils

class TLClassifier(object):
    def __init__(self):       
        #TODO load classifier
        # pass
        num_classes = 4
        pwd = os.path.dirname(os.path.realpath(__file__))
        model_path = os.path.join(pwd, 'models/sim_graph_ssd.pb')
        labeltxt_path = os.path.join(pwd, 'labels_map.pbtxt')
        
        label_map = label_utils.load_labelmap(labeltxt_path)
        self.category = label_utils.convert_label_map_to_categories(label_map, max_num_classes=num_classes, use_display_name=True)
        self.category_idx = label_utils.create_category_index(self.category)
        
        self.detection_graph = tf.Graph()
        
        with self.detection_graph.as_default():
            graph_def = tf.GraphDef()
            with tf.gfile.GFile(model_path, 'rb') as g_file:
                read_graph = g_file.read()
                graph_def.ParseFromString(read_graph)
                tf.import_graph_def(graph_def, name='')
            self.sess = tf.Session(graph=self.detection_graph)

        self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
        self.detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
        self.detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
        self.num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')
        self.detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        
        with self.detection_graph.as_default():
            expanded_img = np.expand_dims(image, axis=0)

            (boxes, scores, classes, num) = self.sess.run([self.detection_boxes,self.detection_scores, 
                                                           self.detection_classes,self.num_detections],
                                                           feed_dict={self.image_tensor: expanded_img})
            boxes = np.squeeze(boxes)
            classes = np.squeeze(classes).astype(np.int32)
            scores = np.squeeze(scores)
            traffic_light = TrafficLight.UNKNOWN
            min_threshold = 0.5
            
            idx = scores.argmax()
            max_score = max(scores)
            if scores[idx] > min_threshold:
                prediction = self.category_idx[classes[idx]]['name']
                rospy.logwarn("[Traffic light detection:] {}".format(prediction))
                if prediction == 'red':
                    traffic_light = TrafficLight.RED
                elif prediction == 'green':
                    traffic_light = TrafficLight.GREEN
                elif prediction == 'yellow':
                    traffic_light = TrafficLight.YELLOW
            else:
                # rospy.logwarn("[No Result]")
                            
        return traffic_light
