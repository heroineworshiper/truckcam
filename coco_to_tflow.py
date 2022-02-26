#!/usr/bin/python3

# Convert objects in 1 category to a tensorflow training set.
# Optionally stretch the aspect ratio for animorphic video.
# The category ID is hard coded for the input category & output category.
# The COCO parser is copied from https://github.com/immersive-limit/coco-manager

# Run this in the same virtual python environment that you use for
# training tensorflow so it can access opencv.
# source /root/yolov5/YoloV5_VirEnv/bin/activate

# Create validation set:
# python3 ~/truckcam/coco_to_tflow.py annotations/instances_val2017.json val2017/ val_person_200 200

# Create training set:
# python3 ~/truckcam/coco_to_tflow.py annotations/instances_train2017.json train2017/ train_person_1000 1000

# Test training set:
# python3 ~/truckcam/coco_to_tflow.py annotations/instances_train2017.json train2017/ train_person_100 100



import os
import sys
import fileinput
import shutil
import json
from pathlib import Path
import random
import cv2 as cv

old_category = 1
new_category = 'Lion'
max_objects = -1


# Scale X coordinates by this amount for animorphic video
# This doesn't work.
#stretch = 9.0 / 16.0
# don't scale
stretch = 1.0



# minimum size of object as a fraction of the image height
min_size = 1.0 / 4.0



class ImageObject:
    def __init__(self):
        self.category_id = -1
        self.box = [ 0.0, 0.0, 0.0, 0.0 ]



images = []

class Coco():
    def _process_info(self):
        self.info = self.coco['info']
        
    def _process_licenses(self):
        self.licenses = self.coco['licenses']
        
    def _process_categories(self):
        self.categories = dict()
        self.super_categories = dict()
        self.category_set = set()

        for category in self.coco['categories']:
            cat_id = category['id']
            super_category = category['supercategory']
            
            # Add category to categories dict
            if cat_id not in self.categories:
                self.categories[cat_id] = category
                self.category_set.add(category['name'])
            else:
                print(f'ERROR: Skipping duplicate category id: {category}')
            
            # Add category id to the super_categories dict
            if super_category not in self.super_categories:
                self.super_categories[super_category] = {cat_id}
            else:
                self.super_categories[super_category] |= {cat_id} # e.g. {1, 2, 3} |= {4} => {1, 2, 3, 4}

    def _process_images(self):
        self.images = dict()
        for image in self.coco['images']:
            image_id = image['id']
            if image_id not in self.images:
# create a new image record
                self.images[image_id] = image
            else:
                print(f'ERROR: Skipping duplicate image id: {image}')

    def _process_segmentations(self):
        self.segmentations = dict()
        for segmentation in self.coco['annotations']:
            image_id = segmentation['image_id']
            if image_id not in self.segmentations:
                self.segmentations[image_id] = []
            self.segmentations[image_id].append(segmentation)




    def process(self, input_json, input_dir, output_dir):
        # Open json
        self.input_json_path = Path(input_json)

        if not self.input_json_path.exists():
            print('Input JSON path not found.')
            print('Giving up & going to a movie.')
            quit()

        # Load the json into RAM
        print('Loading JSON file...')
        with open(self.input_json_path) as json_file:
            self.coco = json.load(json_file)

        print('Processing input JSON...')
        self._process_info()
        self._process_licenses()
        # category ID -> category
        self._process_categories()
        # image ID -> image file
        self._process_images()
        # bounding boxes -> image, category
        self._process_segmentations()

# Want a .xml file for each image with the desired objects in it
        total_src_images = len(self.images.items())
        print("total_src_images=%d" % total_src_images)

# count the old objects
        total_src_objects = 0
        for key, objects in self.segmentations.items():
            total_src_objects += len(objects)
        print("total_src_objects=%d" % total_src_objects)

# randomize all the source images
        keys = list(self.segmentations.keys())
        random.shuffle(keys)

# read the old objects
        count = 0
        new_images = dict()
# 1 segmentation key for each image with multiple objects in it
        for key in keys:
            objects = self.segmentations[key]
            #print("seg=%s, %d" % (key, len(objects)))
            for object in objects:
                if object['category_id'] == old_category and \
                    object['iscrowd'] == 0:
                    image_id = object['image_id']
                    image = self.images[image_id]
                    image_w = image['width']
                    image_h = image['height']
                    # bbox is x, y, w, h
                    object_x = object['bbox'][0]
                    object_y = object['bbox'][1]
                    object_w = object['bbox'][2]
                    object_h = object['bbox'][3]

# object must be big enough
                    if float(object_h) / float(image_h) >= min_size:

# create the new image annotation
                        image_object = ImageObject()
                        image_object.category_id = old_category
# tflow expects the xmin, ymin, xmax, ymax in pixels
                        image_object.box = [int(object_x),
                            int(object_y),
                            int(object_x + object_w),
                            int(object_y + object_h)]
                        if image_id not in new_images:
                            new_images[image_id] = []
                        new_images[image_id].append(image_object)
                        count += 1
                if max_objects > 0 and count > max_objects:
                    break
            if max_objects > 0 and count > max_objects:
                break

        print("new images=%d" % len(new_images.items()))
        print("new objects=%d" % count)
        print("Writing images")
        
        if True:
            for key, objects in new_images.items():
                image = self.images[key]

                prefix = image['file_name'].split('.')[0]
                xml_filename = prefix + '.xml'

#                print('%s' % image)

                old_path = os.path.join(input_dir, image['file_name'])
                new_path = os.path.join(output_dir, image['file_name'])
                xml_path = os.path.join(output_dir, xml_filename)

                xscale = 1.0
                yscale = 1.0

# copy it directly if not stretching
                if abs(stretch - 1.0) < 0.01:
                    shutil.copy(old_path, output_dir)
                else:
# shrink the image to animorphic
                    width = image['width'] * stretch
                    height = image['height']
                    xscale = stretch
                    yscale = 1.0
                    scale = 1.0

# expand the image to 640 on the longest side
                    max_dimension = 640
                    if width < height:
                        scale = max_dimension / height
                    else:
                        scale = max_dimension / width
                    xscale *= scale
                    yscale *= scale

# shrink it again
                width = int(image['width'] * xscale)
                height = int(image['height'] * yscale)

                img = cv.imread(old_path)
                resized = cv.resize(img,
                    None,
                    fx=xscale, 
                    fy=yscale, 
                    interpolation = cv.INTER_LINEAR)
                cv.imwrite(new_path,
                    resized,
                    [int(cv.IMWRITE_JPEG_QUALITY), 90])

# copy the image to the destination dir
#                print('old_path=%s output_dir=%s' % (old_path, output_dir))
#                shutil.copy(old_path, output_dir)

# create the XML annotations
                file = open(xml_path, 'w')
                
                
                file.write('<annotation>\n' + 
                    '\t<filename>' + image['file_name'] + '</filename>\n' + 
                    '\t<path>' + os.path.abspath(new_path) + '</path>\n' + 
                    '\t<source><database>Unknown</database></source>\n' + 
                    '\t<size>\n' + 
                    '\t\t<width>' + str(width) + '</width>' + 
                    '<height>' + str(height) + '</height>' + 
                    '<depth>3</depth>\n' + 
                    '\t</size>\n' +
                    '\t<segmented>0</segmented>\n')
                    
                
                for i in objects:
# scale the box to animorphic
                    x1 = int(i.box[0] * xscale)
                    x2 = int(i.box[2] * xscale)
                    y1 = int(i.box[1] * yscale)
                    y2 = int(i.box[3] * yscale)
                    file.write('\t<object>\n' +
                        '\t\t<name>' + new_category + '</name>\n' +
                        '\t\t<pose>Unspecified</pose>\n' +
                        '\t\t<truncated>0</truncated>\n' +
                        '\t\t<difficult>0</difficult>\n' +
                        '\t\t<bndbox>\n' +
                        '\t\t\t<xmin>' + str(x1) + '</xmin>' + 
                        '<ymin>' + str(y1) + '</ymin>' +
                        '<xmax>' + str(x2) + '</xmax>' +
                        '<ymax>' + str(y2) + '</ymax>\n' +
                        '\t\t</bndbox>\n' +
                        '\t</object>\n')
                file.write('</annotation>\n')
                file.close()

if __name__ == "__main__":
    input_json = sys.argv[1]
    input_dir = sys.argv[2]
    output_dir = sys.argv[3]
    max_objects = int(sys.argv[4])

    print("JSON file: %s" % input_json)
    print("Source image dir: %s" % input_dir)
    print("Destination dir: %s" % output_dir)
    print("Source category: %d" % old_category)
    print("Dest category: %s" % new_category)
    print("Total objects: %d" % max_objects)
    print("Minimum object height: %f" % min_size)
    print("X Stretch: %f" % stretch)

    value = input('Press Enter to continue.')

    cf = Coco()
    cf.process(input_json, input_dir, output_dir)





