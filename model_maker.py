# This generates a tensorflow object detector for 1 object type, 
# from a bunch of images

# The arguments are hard coded.  

# Enter the virtual python environment
# source /root/yolov5/YoloV5_VirEnv/bin/activate


# Run it as
# time LD_LIBRARY_PATH=/usr/local/cuda-11.2/targets/x86_64-linux/lib/ python3 model_maker.py

import numpy as np
import os


#TRAIN_DATA = '../train_person_1000'
#VAL_DATA = '../val_person_200'
#OUTPUT = 'efficientlion0.tflite.1000.300epoch'
#EPOCHS = 300
#MODEL = 'efficientdet_lite0'

TRAIN_DATA = '../train_lion'
VAL_DATA = '../val_lion'
OUTPUT = 'efficientlion0.yolo.100.tflite'
EPOCHS = 100
MODEL = 'efficientdet_lite0'

CATEGORY = 'Lion'
BATCH_SIZE = 4



from tflite_model_maker.config import ExportFormat, QuantizationConfig
from tflite_model_maker import model_spec
from tflite_model_maker import object_detector

from tflite_support import metadata

import tensorflow as tf
assert tf.__version__.startswith('2')

tf.get_logger().setLevel('ERROR')
from absl import logging
logging.set_verbosity(logging.ERROR)

 
# dynamically allocate GPU memory instead of reserving all of it
#gpus = tf.config.experimental.list_physical_devices('GPU')
#if gpus:
#    try:
#         for gpu in gpus:
#              tf.config.experimental.set_memory_growth(gpu, True)
#    except RuntimeError as e:
#        print(e)





train_data = object_detector.DataLoader.from_pascal_voc(
    TRAIN_DATA,
    TRAIN_DATA,
    ['Lion']
)

val_data = object_detector.DataLoader.from_pascal_voc(
    VAL_DATA,
    VAL_DATA,
    ['Lion']
)

spec = model_spec.get(MODEL)

model = object_detector.create(train_data, 
    model_spec=spec, 
    batch_size=BATCH_SIZE, 
    train_whole_model=True, 
    epochs=EPOCHS, 
    validation_data=val_data)

model.evaluate(val_data)

model.export(export_dir='.', tflite_filename=OUTPUT)





