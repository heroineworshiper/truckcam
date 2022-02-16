#!/usr/bin/python3

# Convert all the objects in 1 category to tensorflow XML annotations.
# The category ID is hard coded for the input category & output category.
# The COCO parser is copied from https://github.com/immersive-limit/coco-manager

# set max_objects for testing on a smaller data set

# python3 ~/truckcam/coco_to_tflow.py annotations/instances_val2017.json val2017/ val_person_100
# python3 ~/truckcam/coco_to_tflow.py annotations/instances_train2017.json train2017/ train_person_500

import os
import sys
import fileinput
import shutil
import json
from pathlib import Path
import random

old_category = 1
new_category = 'Lion'
# limit the maximum number of objects for testing
#max_objects = -1
max_objects = 500

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
        for key, value in self.segmentations.items():
            total_src_objects += len(value)
        print("total_src_objects=%d" % total_src_objects)

# randomize the source objects
        print("Randomizing source objects")
        keys = list(self.segmentations.keys())
        random.shuffle(keys)

# read the old objects
        count = 0
        new_images = dict()
# 1 segmentation key for each image with multiple objects in it
        for key in keys:
            value = self.segmentations[key]
            #print("seg=%s, %d" % (key, len(value)))
            for object in value:
                if object['category_id'] == old_category and object['iscrowd'] == 0:
                    image_id = object['image_id']
                    image = self.images[image_id]
                    image_w = image['width']
                    image_h = image['height']
                    # bbox is x, y, w, h
                    object_x = object['bbox'][0]
                    object_y = object['bbox'][1]
                    object_w = object['bbox'][2]
                    object_h = object['bbox'][3]

#                    if image_id == 370210:
#                        print("seg=%s" % object)
#                        print("image=%s w=%d h=%d box=%d,%d,%d,%d" % 
#                            (image['file_name'], image_w, image_h, object_x, object_y, object_w, object_h))
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
# debug
                if max_objects > 0 and count > max_objects:
                    break
            if max_objects > 0 and count > max_objects:
                break

        print("new images=%d" % len(new_images.items()))
        print("new objects=%d" % count)
        print("Writing images")
        
        if True:
            for key, value in new_images.items():
                image = self.images[key]

                prefix = image['file_name'].split('.')[0]
                xml_filename = prefix + '.xml'

#                print('%s' % image)

                old_path = os.path.join(input_dir, image['file_name'])
                xml_path = os.path.join(output_dir, xml_filename)
                jpg_path = os.path.join(output_dir, image['file_name'])
# copy the image to the destination dir
#                print('old_path=%s output_dir=%s' % (old_path, output_dir))
                shutil.copy(old_path, output_dir)

# create the XML annotations
                file = open(xml_path, 'w')
                file.write('<annotation>\n' + 
                    '\t<filename>' + image['file_name'] + '</filename>\n' + 
                    '\t<path>' + os.path.abspath(jpg_path) + '</path>\n' + 
                    '\t<source><database>Unknown</database></source>\n' + 
                    '\t<size>\n' + 
                    '\t\t<width>' + str(image['width']) + '</width>' + 
                    '<height>' + str(image['height']) + '</height>' + 
                    '<depth>3</depth>\n' + 
                    '\t</size>\n' +
                    '\t<segmented>0</segmented>\n')
                    
                
                for i in value:
                    file.write('\t<object>\n' +
                        '\t\t<name>' + new_category + '</name>\n' +
                        '\t\t<pose>Unspecified</pose>\n' +
                        '\t\t<truncated>0</truncated>\n' +
                        '\t\t<difficult>0</difficult>\n' +
                        '\t\t<bndbox>\n' +
                        '\t\t\t<xmin>' + str(i.box[0]) + '</xmin>' + 
                        '<ymin>' + str(i.box[1]) + '</ymin>' +
                        '<xmax>' + str(i.box[2]) + '</xmax>' +
                        '<ymax>' + str(i.box[3]) + '</ymax>\n' +
                        '\t\t</bndbox>\n' +
                        '\t</object>\n')
                file.write('</annotation>\n')
                file.close()

if __name__ == "__main__":
    input_json = sys.argv[1]
    input_dir = sys.argv[2]
    output_dir = sys.argv[3]

    print("JSON file: %s" % input_json)
    print("Source image dir: %s" % input_dir)
    print("Destination dir: %s" % output_dir)
    print("Source category ID: %d" % old_category)
    print("Dest category index: %s" % new_category)
    print("Max objects: %d" % max_objects)
    
    value = input('Press Enter to continue.')

    cf = Coco()
    cf.process(input_json, input_dir, output_dir)





