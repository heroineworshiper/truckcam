# This generates a tensorflow object detector for 1 object type,
# from a bunch of images

# The arguments are hard coded.  Run it as
# LD_LIBRARY_PATH=/usr/local/cuda-11.2/targets/x86_64-linux/lib/ python3 model_maker.py

import numpy as np
import os

TRAIN_DATA = '../train_person_1000'
VAL_DATA = '../val_person_1000'
CATEGORY = 'Lion'

from tflite_model_maker.config import ExportFormat, QuantizationConfig
from tflite_model_maker import model_spec
from tflite_model_maker import object_detector

from tflite_support import metadata

import tensorflow as tf
assert tf.__version__.startswith('2')

tf.get_logger().setLevel('ERROR')
from absl import logging
logging.set_verbosity(logging.ERROR)


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

spec = model_spec.get('efficientdet_lite0')

model = object_detector.create(train_data, 
    model_spec=spec, 
    batch_size=4, 
    train_whole_model=True, 
    epochs=50, 
    validation_data=val_data)

model.evaluate(val_data)

model.export(export_dir='.', tflite_filename='model.tflite')





