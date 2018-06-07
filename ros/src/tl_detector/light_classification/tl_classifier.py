from styx_msgs.msg import TrafficLight

import os


class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        # pass
        num_classes = 3
        pwd = os.path.dirname(os.path.realpath(__file__))
        model_path = os.path.join(pwd, '')
        labeltxt_path = os.path.join(pwd, '')

        self.img = None
        self.category_dict =  { 1:{'name':'red', 'id':1}, 
                                2:{'name':'yellow', 'id':2}, 
                                3:{'name':'green', 'id':3}}
        




    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        
        traffic_light = TrafficLight.UNKNOWN
        prediction
        if prediction == 'red':
            traffic_light = TrafficLight.RED
        elif prediction == 'green':
            traffic_light = TrafficLight.GREEN
        elif prediction == 'YELLOW':
            traffic_light = TrafficLight.YELLOW
        
        return traffic_light
