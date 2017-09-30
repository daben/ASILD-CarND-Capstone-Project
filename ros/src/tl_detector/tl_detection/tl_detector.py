from __future__ import division
from __future__ import print_function

import rospy
from styx_msgs.msg import TrafficLight
import tensorflow as tf
import numpy as np
import os


TRAFFIC_LIGHT_CLASSES = (
    None,
    TrafficLight.RED,
    TrafficLight.YELLOW,
    TrafficLight.GREEN,
    TrafficLight.UNKNOWN  # OFF
)

DEFAULT_MODEL_PATH = \
    os.path.join(os.path.dirname(__file__),
                 "models/frozen_inference_graph.pb")


class TrafficLightDetector(object):
    def __init__(self, model=DEFAULT_MODEL_PATH, score_threshold=.5):
        rospy.logdebug("tl_detector loading model %s", model)
        self.min_score_threshold = score_threshold
        self.detection_graph = self._load_inference_graph(model)
        self._prepare_session(self.detection_graph)
        # warm up
        self.analyze(np.zeros((64,64,3), dtype=np.uint8))

    def analyze(self, image):
        """Detects and classify traffic lights in an image

        Args:
            image (ndarray): image

        Returns:
            tuple: result of the detection
        """

        image_batch = image[np.newaxis]
        result = self.session.run(self.inference_op,
                                  feed_dict={self.image_tensor: image_batch})
        scores, boxes, classes = map(lambda x: x[0], result)
        top_score = scores[0]
        if 0.3 < top_score < self.min_score_threshold:
            indices = scores > 0.3
        else:
            indices = scores > self.min_score_threshold
        scores = scores[indices]
        boxes = boxes[indices]
        classes = classes[indices]
        classes = tuple(TRAFFIC_LIGHT_CLASSES[int(_)] for _ in classes)

        return tuple(zip(classes, scores, boxes))

    def _prepare_session(self, detection_graph):
        self.session = tf.Session(graph=detection_graph)

        self.image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')
        # Each box represents a part of the image where a particular object was detected.
        detection_boxes = detection_graph.get_tensor_by_name('detection_boxes:0')
        # Each score represent how level of confidence for each of the objects.
        # Score is shown on the result image, together with the class label.
        detection_scores = detection_graph.get_tensor_by_name('detection_scores:0')
        detection_classes = detection_graph.get_tensor_by_name('detection_classes:0')
        # num_detections = detection_graph.get_tensor_by_name('num_detections:0')

        self.inference_op = [detection_scores, detection_boxes, detection_classes]

    def _load_inference_graph(self, filename):
        detection_graph = tf.Graph()
        with detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(filename, 'rb') as fd:
                s_graph = fd.read()
                od_graph_def.ParseFromString(s_graph)
                tf.import_graph_def(od_graph_def, name='')
        return detection_graph


if __name__ == '__main__':

    import argparse
    import PIL.Image
    import time

    flags = argparse.ArgumentParser(description="Traffic Light Detector")
    flags.add_argument("images", metavar="IMAGE", type=str, nargs="+")
    flags.add_argument("--score_threshold", type=float, default=0.5)

    flags = flags.parse_args()

    state_map = {0: "RED", 1: "YELLOW", 2: "GREEN", 4: "UNKNOWN"}

    tl_detector = TrafficLightDetector(score_threshold=flags.score_threshold)

    for image_path in flags.images:
        image = np.array(PIL.Image.open(image_path))[..., :3]

        elapsed = time.clock()
        results = tl_detector.analyze(image)
        elapsed = time.clock() - elapsed

        if elapsed > 0.1:
            elapsed_string = "{:.1f} s".format(elapsed)
        else:
            elapsed_string = "{:.1f} ms".format(elapsed * 1e3)

        print(image_path, "({})".format(elapsed_string))

        if results:
            for state, confidence, box in results:
                state_str = state_map.get(state, "UNDEFINED(%d)" % state)
                print("  - {:6s} ({:3.0f}%)  {}".format(state_str, confidence*100, box))
        else:
            print("  - NONE")
