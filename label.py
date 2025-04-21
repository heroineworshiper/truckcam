# YOLOv5 🚀 by Ultralytics, GPL-3.0 license
"""
Label just the lions.

Output XML files or JSON for training tensorflow.

Copies the images to some hard coded directories.

# Download the YOLO model from
# https://github.com/ultralytics/yolov5/releases/tag/v6.1

# Enter the virtual python environment.  
source YoloV5_VirEnv/bin/activate

# Run it as 
python3 label.py --weights yolov5x6.pt


"""

# total validation images.
TOTAL_VAL = 100
#TOTAL_VAL = 10
# total training images
TOTAL_TRAIN = 1000
#TOTAL_TRAIN = 10
# source for the images
SRC_DIR = '../lion_images2/'
# destination for the annotations & images
VAL_DIR = '../val_lion/'
TRAIN_DIR = '../train_lion/'
WANT_CLS = 0  # person ID
NEW_CATEGORY = 'lion'

# annotation format
XML_ANNOTATION = False  # for model_maker
JSON_ANNOTATION = True  # for efficientdet
# destination for JSON annotations
TRAIN_ANNOTATIONS = '../train_lion/instances_train.json'
VAL_ANNOTATIONS = '../val_lion/instances_val.json'

# output image size if scaling
#DST_W = 640
#DST_H = 640
# output image size if passing through
DST_W = -1
DST_H = -1


import argparse
import os
import sys
import random
from pathlib import Path
import glob
import shutil

import cv2
import numpy as np
import torch
import torch.backends.cudnn as cudnn

FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]  # YOLOv5 root directory
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative

from models.common import DetectMultiBackend
from utils.datasets import IMG_FORMATS, VID_FORMATS, LoadImages, LoadStreams
from utils.general import (LOGGER, check_file, check_img_size, check_imshow, check_requirements, colorstr,
                           increment_path, non_max_suppression, print_args, scale_coords, strip_optimizer, xyxy2xywh)
from utils.plots import Annotator, colors, save_one_box
from utils.torch_utils import select_device, time_sync
from utils.augmentations import Albumentations, augment_hsv, copy_paste, letterbox, mixup, random_perspective

dst_train_files = []
dst_train_rects = []
dst_val_files = []
dst_val_rects = []




class LoadImages2:
    def __init__(self, path, img_size=640, stride=32, auto=True):
        #print("LoadImages2 path=%s" % path)
        p = str(Path(path).resolve())  # os-agnostic absolute path
        files = sorted(glob.glob(os.path.join(p, '*.*')))
        random.shuffle(files)

        #print("LoadImages2 %s" % str(files))
        images = [x for x in files]
        ni, nv = len(images), 0

        self.img_size = img_size
        self.stride = stride
        self.files = images
        self.nf = ni + nv  # number of files
        self.video_flag = [False] * ni + [True] * nv
        self.mode = 'image'
        self.auto = auto
        self.cap = None

    def __iter__(self):
        self.count = 0
        return self

    def __next__(self):
        if self.count == self.nf:
            raise StopIteration
        path = self.files[self.count]

        # Read image
        self.count += 1
        img0 = cv2.imread(path)  # BGR
        assert img0 is not None, f'Image Not Found {path}'
        s = f'image {self.count}/{self.nf} {path}: '

        # Padded resize
        img = letterbox(img0, self.img_size, stride=self.stride, auto=self.auto)[0]

        # Convert
        img = img.transpose((2, 0, 1))[::-1]  # HWC to CHW, BGR to RGB
        img = np.ascontiguousarray(img)

        return path, img, img0, self.cap, s


    def __len__(self):
        return self.nf  # number of files




@torch.no_grad()
def run(weights=ROOT / 'yolov5s.pt',  # model.pt path(s)
        data=ROOT / 'data/coco128.yaml',  # dataset.yaml path
        imgsz=(640, 640),  # inference size (height, width)
        conf_thres=0.25,  # confidence threshold
        iou_thres=0.45,  # NMS IOU threshold
        max_det=1000,  # maximum detections per image
        device='',  # cuda device, i.e. 0 or 0,1,2,3 or cpu
        view_img=False,  # show results
        save_txt=False,  # save results to *.txt
        save_conf=False,  # save confidences in --save-txt labels
        save_crop=False,  # save cropped prediction boxes
        nosave=False,  # do not save images/videos
        classes=None,  # filter by class: --class 0, or --class 0 2 3
        agnostic_nms=False,  # class-agnostic NMS
        augment=False,  # augmented inference
        visualize=False,  # visualize features
        update=False,  # update all models
        project=ROOT / 'runs/detect',  # save results to project/name
        name='exp',  # save results to project/name
        exist_ok=False,  # existing project/name ok, do not increment
        line_thickness=3,  # bounding box thickness (pixels)
        hide_labels=False,  # hide labels
        hide_conf=False,  # hide confidences
        half=False,  # use FP16 half-precision inference
        dnn=False,  # use OpenCV DNN for ONNX inference
        ):
    source = SRC_DIR
    save_img = not nosave and not source.endswith('.txt')  # save inference images
    is_file = Path(source).suffix[1:] in (IMG_FORMATS + VID_FORMATS)
    is_url = source.lower().startswith(('rtsp://', 'rtmp://', 'http://', 'https://'))
    webcam = source.isnumeric() or source.endswith('.txt') or (is_url and not is_file)
    if is_url and is_file:
        source = check_file(source)  # download

    # Directories
    save_dir = increment_path(Path(project) / name, exist_ok=exist_ok)  # increment run
    (save_dir / 'labels' if save_txt else save_dir).mkdir(parents=True, exist_ok=True)  # make dir

    # Load model
    device = select_device(device)
    model = DetectMultiBackend(weights, device=device, dnn=dnn, data=data)
    stride, names, pt, jit, onnx, engine = model.stride, model.names, model.pt, model.jit, model.onnx, model.engine
    imgsz = check_img_size(imgsz, s=stride)  # check image size

    print('TOTAL_VAL=%d TOTAL_TRAIN=%d\n' % (TOTAL_VAL, TOTAL_TRAIN))

    # Half
    half &= (pt or jit or onnx or engine) and device.type != 'cpu'  # FP16 supported on limited backends with CUDA
    if pt or jit:
        model.model.half() if half else model.model.float()

    # Dataloader
    dataset = LoadImages2(source, img_size=imgsz, stride=stride, auto=pt)
    bs = 1  # batch_size
    vid_path, vid_writer = [None] * bs, [None] * bs


    # Run inference
    model.warmup(imgsz=(1 if pt else bs, 3, *imgsz), half=half)  # warmup
    dt, seen = [0.0, 0.0, 0.0], 0
    counter = 0
    done = False
    for path, im, im0s, vid_cap, s in dataset:
        if done: break

        t1 = time_sync()

# image im is stretched to 640x384 to meet stride requirements


        im = torch.from_numpy(im).to(device)
        im = im.half() if half else im.float()  # uint8 to fp16/32
        im /= 255  # 0 - 255 to 0.0 - 1.0
        if len(im.shape) == 3:
            im = im[None]  # expand for batch dim
        t2 = time_sync()
        dt[0] += t2 - t1

        # Inference
        visualize = increment_path(save_dir / Path(path).stem, mkdir=True) if visualize else False
        pred = model(im, augment=augment, visualize=visualize)
        t3 = time_sync()
        dt[1] += t3 - t2

        # NMS
        pred = non_max_suppression(pred, conf_thres, iou_thres, classes, agnostic_nms, max_det=max_det)
        dt[2] += time_sync() - t3

        # Second-stage classifier (optional)
        # pred = utils.general.apply_classifier(pred, classifier_model, im, im0s)

        # Process predictions
        for i, det in enumerate(pred):  # per image
            if done: break

            seen += 1
            if webcam:  # batch_size >= 1
                p, im0, frame = path[i], im0s[i].copy(), dataset.count
                s += f'{i}: '
            else:
                p, im0, frame = path, im0s.copy(), getattr(dataset, 'frame', 0)

            p = Path(p)  # to Path
            save_path = str(save_dir / p.name)  # im.jpg
            txt_path = str(save_dir / 'labels' / p.stem) + ('' if dataset.mode == 'image' else f'_{frame}')  # im.txt
            s += '%gx%g ' % im.shape[2:]  # print string
            gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]  # normalization gain whwh
            imc = im0.copy() if save_crop else im0  # for save_crop
            annotator = Annotator(im0, line_width=line_thickness, example=str(names))

            if len(det):

# save the training data
                if counter < TOTAL_VAL:
                    is_train = False
                    dst_img = VAL_DIR + p.name
                    dst_xml = VAL_DIR + p.stem + '.xml'
                elif counter < TOTAL_VAL + TOTAL_TRAIN:
                    is_train = True
                    dst_img = TRAIN_DIR + p.name
                    dst_xml = TRAIN_DIR + p.stem + '.xml'
                else:
                    done = True
                    break

#                print('counter=%d path=%s dst_img=%s dst_xml=%s' % 
#                    (counter, path, dst_img, dst_xml));

# get biggest lion
                best_x1 = -1
                best_y1 = -1
                best_x2 = -1
                best_y2 = -1
                best_area = -1
                for *xyxy, cls in det:
                    xywh = torch.tensor(xyxy)
                    w = im.shape[3]
                    h = im.shape[2]
                    img_w = int(gn[0])
                    img_h = int(gn[1])
                    x1 = int(xyxy[0] / w * img_w)
                    y1 = int(xyxy[1] / h * img_h)
                    x2 = int(xyxy[2] / w * img_w)
                    y2 = int(xyxy[3] / h * img_h)
                    c = int(cls)
#                    print("c=%s src=%dx%d dst=%dx%d\nsrc=%d,%d - %d,%d dst=%d,%d - %d,%d" % 
#                        (names[c], w, h, img_w, img_h, xyxy[0], xyxy[1], xyxy[2], xyxy[3], x1, y1, x2, y2))
                    if c == WANT_CLS:
                        area = (x2 - x1) * (y2 - y1)
                        if best_area < 0 or area > best_area:
                            best_area = area
                            best_x1 = x1
                            best_y1 = y1
                            best_x2 = x2
                            best_y2 = y2


                if best_area < 0:
                    print("%s didn't have a lion" % path)
                else:
                
# reload the original image for scaling
                    im_orig = cv2.imread(path)
#                    print("path=%s h=%d w=%d" % (path, im_orig.shape[0], im_orig.shape[1]))
                    w = im_orig.shape[1]
                    h = im_orig.shape[0]

                    global DST_W
                    global DST_H
                    if DST_W < 0: 
                        DST_W = w
                    if DST_H < 0: 
                        DST_H = h
                    rect = [int(best_x1 * DST_W / w), 
                            int(best_y1 * DST_H / h), 
                            int(best_x2 * DST_W / w), 
                            int(best_y2 * DST_H / h)]
# copy the image file
                    if DST_W == w and DST_H == h:
                        shutil.copy(path, dst_img)
                    else:
# scale the image file
                        im_scaled = cv2.resize(im_orig, (DST_W, DST_H))
                        cv2.imwrite(dst_img, im_scaled)
                
                    if XML_ANNOTATION:
# generate the XML
                        file = open(dst_xml, 'w')
                        file.write('<annotation>\n' + 
                            '\t<filename>' + p.name + '</filename>\n' + 
                            '\t<path>' + os.path.abspath(dst_img) + '</path>\n' + 
                            '\t<source><database>Unknown</database></source>\n' + 
                            '\t<size>\n' + 
                            '\t\t<width>' + str(DST_W) + '</width>' + 
                            '<height>' + str(DST_H) + '</height>' + 
                            '<depth>3</depth>\n' + 
                            '\t</size>\n' +
                            '\t<segmented>0</segmented>\n')
                        file.write('\t<object>\n' +
                            '\t\t<name>' + NEW_CATEGORY + '</name>\n' +
                            '\t\t<pose>Unspecified</pose>\n' +
                            '\t\t<truncated>0</truncated>\n' +
                            '\t\t<difficult>0</difficult>\n' +
                            '\t\t<bndbox>\n' +
                            '\t\t\t<xmin>' + str(rect[0]) + '</xmin>' + 
                            '<ymin>' + str(rect[1]) + '</ymin>' +
                            '<xmax>' + str(rect[2]) + '</xmax>' +
                            '<ymax>' + str(rect[3]) + '</ymax>\n' +
                            '\t\t</bndbox>\n' +
                            '\t</object>\n')
                        file.write('</annotation>\n')
                        file.close()
                    else:
# generate the JSON
                        if is_train:
                            dst_train_files.append(p.name)
                            dst_train_rects.append(rect)
                        else:
                            dst_val_files.append(p.name)
                            dst_val_rects.append(rect)

                    counter += 1



                # Rescale boxes from img_size to im0 size
                det[:, :4] = scale_coords(im.shape[2:], det[:, :4], im0.shape).round()

                # Print results
                for c in det[:, -1].unique():
                    n = (det[:, -1] == c).sum()  # detections per class
                    s += f"{n} {names[int(c)]}{'s' * (n > 1)}, "  # add to string

                # Write results
                for *xyxy, conf, cls in reversed(det):
                    if save_txt:  # Write to file
                        xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4)) / gn).view(-1).tolist()  # normalized xywh
                        line = (cls, *xywh, conf) if save_conf else (cls, *xywh)  # label format
                        with open(txt_path + '.txt', 'a') as f:
                            f.write(('%g ' * len(line)).rstrip() % line + '\n')

                    if save_img or save_crop or view_img:  # Add bbox to image
                        c = int(cls)  # integer class
                        label = None if hide_labels else (names[c] if hide_conf else f'{names[c]} {conf:.2f}')
                        annotator.box_label(xyxy, label, color=colors(c, True))
                        if save_crop:
                            save_one_box(xyxy, imc, file=save_dir / 'crops' / names[c] / f'{p.stem}.jpg', BGR=True)

            # Stream results
            im0 = annotator.result()
            if view_img:
                cv2.imshow(str(p), im0)
                cv2.waitKey(1)  # 1 millisecond

            # Save results (image with detections)
            if save_img:
                if dataset.mode == 'image':
                    cv2.imwrite(save_path, im0)
                else:  # 'video' or 'stream'
                    if vid_path[i] != save_path:  # new video
                        vid_path[i] = save_path
                        if isinstance(vid_writer[i], cv2.VideoWriter):
                            vid_writer[i].release()  # release previous video writer
                        if vid_cap:  # video
                            fps = vid_cap.get(cv2.CAP_PROP_FPS)
                            w = int(vid_cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                            h = int(vid_cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                        else:  # stream
                            fps, w, h = 30, im0.shape[1], im0.shape[0]
                        save_path = str(Path(save_path).with_suffix('.mp4'))  # force *.mp4 suffix on results videos
                        vid_writer[i] = cv2.VideoWriter(save_path, cv2.VideoWriter_fourcc(*'mp4v'), fps, (w, h))
                    vid_writer[i].write(im0)

        # Print time (inference-only)
        LOGGER.info(f'{s}Done. ({t3 - t2:.3f}s)')

# Save annotation files
# 1 big file for all the annotations
    if JSON_ANNOTATION:
        for i in range(2):
            if i == 0:
                path = VAL_ANNOTATIONS
                dst_files = dst_val_files
                dst_rects = dst_val_rects
            else:
                path = TRAIN_ANNOTATIONS
                dst_files = dst_train_files
                dst_rects = dst_train_rects

            file = open(path, 'w')
            file.write('{\"info\": \n')
            file.write('\t{\"description\": \"\", \"url\": \"\", \"version\": \"\", \"year\": 2020, \"contributor\": \"\", \"date_created\": \"2020-04-14 01:45:07.508229\"}, \n')
            file.write('\t\"licenses\": [{\"id\": 1, \"name\": null, \"url": null}], \n')
            file.write('\t\"categories\": [{\"id\": 1, \"name\": \"lion\", \"supercategory\": \"None\"}], \n')
            file.write('\t\"images\": [\n')

            id = 0
            for name in dst_files:
                file.write('\t\t{\"id\": ' + 
                    str(id) + 
                    ', \"file_name\": \"' + 
                    name + 
                    '\", \"width\": ' + 
                    str(DST_W) + 
                    ', \"height\": ' + 
                    str(DST_H) + 
                    ', \"date_captured\": \"2020-04-14 01:45:07.508146\", \"license\": 1, \"coco_url\": \"\", \"flickr_url\": \"\"}')
                if id < len(dst_files) - 1:
                    file.write(', \n')
                else:
                    file.write('\n')
                id += 1
            file.write('\t], \n')
            file.write('\t\"annotations\": [\n')

            id = 0
            for rect in dst_rects:
                x1 = rect[0]
                y1 = rect[1]
                x2 = rect[2]
                y2 = rect[3]
                w = x2 - x1
                h = y2 - y1
                area = (x2 - x1) * (y2 - y1)
                file.write('\t\t{\"id\": ' + 
                    str(id) + 
                    ', \"image_id\": ' +
                    str(id) +
                    ', \"category_id\": 1, \"iscrowd\": 0, \"area\": ' +
                    str(area) + 
                    ', \n')
                file.write('\t\t\t\"bbox\": [' + str(x1) + ', ' + str(y1) + ', ' + str(w) + ', ' + str(h) + '], \n')
                file.write('\t\t\t\"segmentation\": [[' + str(x1) + ', ' + str(y1) + ', ' + str(x2) + ', ' + str(y1) + ', ' + str(x2) + ', ' + str(y2) + ', ' + str(x1) + ', ' + str(y2) + ']]}')
                if id < len(dst_rects) - 1:
                    file.write(', \n')
                else:
                    file.write('\n')
                id += 1
            file.write('\t]\n')
            file.write('}\n')
            file.close()
            print('Wrote ' + path)




    # Print results
    t = tuple(x / seen * 1E3 for x in dt)  # speeds per image
    LOGGER.info(f'Speed: %.1fms pre-process, %.1fms inference, %.1fms NMS per image at shape {(1, 3, *imgsz)}' % t)
    if save_txt or save_img:
        s = f"\n{len(list(save_dir.glob('labels/*.txt')))} labels saved to {save_dir / 'labels'}" if save_txt else ''
        LOGGER.info(f"Results saved to {colorstr('bold', save_dir)}{s}")
    if update:
        strip_optimizer(weights)  # update model (to fix SourceChangeWarning)


def parse_opt():
    parser = argparse.ArgumentParser()
    parser.add_argument('--weights', nargs='+', type=str, default=ROOT / 'yolov5s.pt', help='model path(s)')
    parser.add_argument('--data', type=str, default=ROOT / 'data/coco128.yaml', help='(optional) dataset.yaml path')
    parser.add_argument('--imgsz', '--img', '--img-size', nargs='+', type=int, default=[640], help='inference size h,w')
    parser.add_argument('--conf-thres', type=float, default=0.25, help='confidence threshold')
    parser.add_argument('--iou-thres', type=float, default=0.45, help='NMS IoU threshold')
    parser.add_argument('--max-det', type=int, default=1000, help='maximum detections per image')
    parser.add_argument('--device', default='', help='cuda device, i.e. 0 or 0,1,2,3 or cpu')
    parser.add_argument('--view-img', action='store_true', help='show results')
    parser.add_argument('--save-txt', action='store_true', help='save results to *.txt')
    parser.add_argument('--save-conf', action='store_true', help='save confidences in --save-txt labels')
    parser.add_argument('--save-crop', action='store_true', help='save cropped prediction boxes')
    parser.add_argument('--nosave', action='store_true', help='do not save images/videos')
    parser.add_argument('--classes', nargs='+', type=int, help='filter by class: --classes 0, or --classes 0 2 3')
    parser.add_argument('--agnostic-nms', action='store_true', help='class-agnostic NMS')
    parser.add_argument('--augment', action='store_true', help='augmented inference')
    parser.add_argument('--visualize', action='store_true', help='visualize features')
    parser.add_argument('--update', action='store_true', help='update all models')
    parser.add_argument('--project', default=ROOT / 'runs/detect', help='save results to project/name')
    parser.add_argument('--name', default='exp', help='save results to project/name')
    parser.add_argument('--exist-ok', action='store_true', help='existing project/name ok, do not increment')
    parser.add_argument('--line-thickness', default=3, type=int, help='bounding box thickness (pixels)')
    parser.add_argument('--hide-labels', default=False, action='store_true', help='hide labels')
    parser.add_argument('--hide-conf', default=False, action='store_true', help='hide confidences')
    parser.add_argument('--half', action='store_true', help='use FP16 half-precision inference')
    parser.add_argument('--dnn', action='store_true', help='use OpenCV DNN for ONNX inference')
    opt = parser.parse_args()
    opt.imgsz *= 2 if len(opt.imgsz) == 1 else 1  # expand
    print_args(FILE.stem, opt)
    return opt


def main(opt):
    check_requirements(exclude=('tensorboard', 'thop'))
    run(**vars(opt))


if __name__ == "__main__":
    opt = parse_opt()
    main(opt)
