from styx_msgs.msg import TrafficLight
import os
import tensorflow as tf
import numpy as np
import rospy

SIM_SSD_GRAPH_FILE = '../../../model/frozen_sim_model/frozen_inference_graph.pb'

class TLClassifier(object):
    def __init__(self):
        self.detection_graph = self.load_graph(SIM_SSD_GRAPH_FILE)
        self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
        self.detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
        self.detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')

        self.current_light_state = TrafficLight.UNKNOWN
        pass

    def load_graph(self, graph_file):
        """Loads a frozen inference graph"""
        graph = tf.Graph()
        with graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(graph_file, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')
                
        return graph

    def filter_boxes(self, min_score, scores, classes):
        """Return boxes with a confidence >= `min_score`"""
        n = len(classes)
        idxs = []
        for i in range(n):
            if scores[i] >= min_score:
                idxs.append(i)
        
        filtered_scores = scores[idxs, ...]
        filtered_classes = classes[idxs, ...]
        return filtered_scores, filtered_classes


    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        expanded_image = np.expand_dims(np.asarray(image, dtype=np.uint8), 0)

        with tf.Session(graph=self.detection_graph) as sess:
            # Actual detection.
            (scores, classes) = sess.run([self.detection_scores,
                self.detection_classes],
                feed_dict={self.image_tensor: expanded_image})

        # Remove unnecessary dimensions
        scores = np.squeeze(scores)
        classes = np.squeeze(classes)

        confidence_cutoff = 0.6
        # Filter boxes with a confidence score less than `confidence_cutoff`
        scores, classes = self.filter_boxes(confidence_cutoff, scores, classes)
        print(scores)
        print(classes)

        if classes.size != 0:
            if classes[0] == 1:
                self.current_light_state = TrafficLight.GREEN
            elif classes[0] == 2:
                self.current_light_state = TrafficLight.RED
            elif classes[0] == 3:
                self.current_light_state = TrafficLight.YELLOW
            else:
                self.current_light_state = TrafficLight.UNKNOWN
            rospy.logwarn("Current light state: {0}".format(classes[0]))
            rospy.logwarn("Current light score: {0}".format(scores[0]))
        else:
            # if no traffic light information can be given with high precision, always stop
            rospy.logwarn("No traffic light information given since probability is smaller than {0}".format(confidence_cutoff))
            self.current_light_state = TrafficLight.UNKNOWN

        return self.current_light_state
