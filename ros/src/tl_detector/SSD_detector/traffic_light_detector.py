import numpy as np
import os
import sys
import tensorflow as tf
import time

from collections import defaultdict
from io import StringIO
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from PIL import Image
from glob import glob
from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as vis_util

def load_image_into_numpy_array(image):
    (im_width, im_height) = image.size
    return np.array(image.getdata()).reshape((im_height, im_width, 3)).astype(np.uint8)


sim_model  = 'trained_ssd_model/frozen_graph_for_sim.pb'
# real_model = 'trained_ssd_model/frozen_graph_for_real.pb'

PATH_TO_LABELS = 'label_map.pbtxt'

# NUM_CLASSES = 14

label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
category_index = label_map_util.create_category_index(categories)
print(category_index)


## 1.
d_graph = tf.Graph()

with d_graph.as_default():
    graph_def = tf.GraphDef()
    with tf.gfile.GFile(sim_model, 'rb') as gf:
        read_graph = gf.read()
        graph_def.ParseFromString(read_graph)
        tf.import_graph_def(graph_def, name='')

TEST_PATH = "test_imgs_from_sim"
# TEST_PATH = "test_imgs_from_real"
# print(os.path.join(TEST_PATH,'*.jpg'))

TEST_IMGS = glob(os.path.join(TEST_PATH, '*.jpg'))
# print('num of test images: ', len(TEST_IMGS))

IMG_SIZE = (12,8)

#-------------------------------------------------------------------------------------------------------
with d_graph.as_default():
    with tf.Session(graph=d_graph) as sess:
        image_tensor = d_graph.get_tensor_by_name('image_tensor:0')
        detection_boxes = d_graph.get_tensor_by_name('detection_boxes:0')
        detection_scores = d_graph.get_tensor_by_name('detection_scores:0')
        detection_classes = d_graph.get_tensor_by_name('detection_classes:0')
        num_detections = d_graph.get_tensor_by_name('num_detections:0')

        for img_path in TEST_IMGS:
            img = Image.open(img_path)
            np_img = load_image_into_numpy_array(img)
            expanded_img = np.expand_dims(np_img, axis=0)
            
            # if there are complied error, to update your tensorflow version may work^-^
            (boxes, scores, classes, num) = sess.run([detection_boxes, detection_scores, detection_classes, num_detections], 
                                                    feed_dict={image_tensor: expanded_img})

            boxes = np.squeeze(boxes)
            scores = np.squeeze(scores)
            classes = np.squeeze(classes)
            
            vis_util.visualize_boxes_and_labels_on_image_array(
                np_img, boxes, classes, scores,
                category_index,
                use_normalized_coordinates=True,
                line_thickness=int(5))
            
            plt.figure(figsize=IMG_SIZE)
            plt.imshow(np_img)
            plt.savefig('output/'+img_path)
            