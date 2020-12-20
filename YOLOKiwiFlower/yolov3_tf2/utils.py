from absl import logging
import numpy as np
import tensorflow as tf
import cv2
import math
from seaborn import color_palette
from PIL import Image, ImageDraw, ImageFont


YOLOV3_LAYER_LIST = [
    'yolo_darknet',
    'yolo_conv_0',
    'yolo_output_0',
    'yolo_conv_1',
    'yolo_output_1',
    'yolo_conv_2',
    'yolo_output_2',
]

YOLOV3_TINY_LAYER_LIST = [
    'yolo_darknet',
    'yolo_conv_0',
    'yolo_output_0',
    'yolo_conv_1',
    'yolo_output_1',
]


def load_darknet_weights(model, weights_file, tiny=False):
    wf = open(weights_file, 'rb')
    major, minor, revision, seen, _ = np.fromfile(wf, dtype=np.int32, count=5)

    if tiny:
        layers = YOLOV3_TINY_LAYER_LIST
    else:
        layers = YOLOV3_LAYER_LIST

    for layer_name in layers:
        sub_model = model.get_layer(layer_name)
        for i, layer in enumerate(sub_model.layers):
            if not layer.name.startswith('conv2d'):
                continue
            batch_norm = None
            if i + 1 < len(sub_model.layers) and \
                    sub_model.layers[i + 1].name.startswith('batch_norm'):
                batch_norm = sub_model.layers[i + 1]

            logging.info("{}/{} {}".format(
                sub_model.name, layer.name, 'bn' if batch_norm else 'bias'))

            filters = layer.filters
            size = layer.kernel_size[0]
            in_dim = layer.input_shape[-1]

            if batch_norm is None:
                conv_bias = np.fromfile(wf, dtype=np.float32, count=filters)
            else:
                bn_weights = np.fromfile(
                    wf, dtype=np.float32, count=4 * filters)
                bn_weights = bn_weights.reshape((4, filters))[[1, 0, 2, 3]]

            conv_shape = (filters, in_dim, size, size)
            conv_weights = np.fromfile(
                wf, dtype=np.float32, count=np.product(conv_shape))
            conv_weights = conv_weights.reshape(
                conv_shape).transpose([2, 3, 1, 0])

            if batch_norm is None:
                layer.set_weights([conv_weights, conv_bias])
            else:
                layer.set_weights([conv_weights])
                batch_norm.set_weights(bn_weights)

    assert len(wf.read()) == 0, 'failed to read all data'
    wf.close()


def broadcast_iou(box_1, box_2):
    # broadcast boxes
    box_1 = tf.expand_dims(box_1, -2)
    box_2 = tf.expand_dims(box_2, 0)

    new_shape = tf.broadcast_dynamic_shape(tf.shape(box_1), tf.shape(box_2))
    box_1 = tf.broadcast_to(box_1, new_shape)
    box_2 = tf.broadcast_to(box_2, new_shape)

    int_w = tf.maximum(tf.minimum(box_1[..., 2], box_2[..., 2]) -
                       tf.maximum(box_1[..., 0], box_2[..., 0]), 0)
    int_h = tf.maximum(tf.minimum(box_1[..., 3], box_2[..., 3]) -
                       tf.maximum(box_1[..., 1], box_2[..., 1]), 0)
    int_area = int_w * int_h
    box_1_area = (box_1[..., 2] - box_1[..., 0]) * \
        (box_1[..., 3] - box_1[..., 1])
    box_2_area = (box_2[..., 2] - box_2[..., 0]) * \
        (box_2[..., 3] - box_2[..., 1])
    return int_area / (box_1_area + box_2_area - int_area)


def distance_point(point_1, point_2):
    # Distancia entre dos puntos
    x1 = point_1[0]
    y1 = point_1[1]
    x2 = point_2[0]
    y2 = point_2[1]
    sum_2 = (x2 - x1) ** 2 + (y2 - y1) ** 2
    return math.sqrt(sum_2)


def flower_position(fig, center, center_pos_box, diag_lenght):
    # X:
    coord_y = int((center[0] - center_pos_box[0]) * 0.48) / 1.e3
    # Y:
    coord_z = int((center[1] - center_pos_box[1]) * 0.48) / 1.e3
    # Z:
    coord_x = int(-0.000011 * (diag_lenght ** 3) + 0.014786 * (diag_lenght ** 2) - 7.011248 * (diag_lenght) + 1332.346686) / 1.e3  # cubic regresion

    return (coord_x, coord_y, coord_z)


def draw_outputs(img, outputs, class_names, fig, pos_flowers, center):

    boxes, objectness, classes, nums = outputs
    boxes, objectness, classes, nums = boxes[0], objectness[0], classes[0], nums[0]
    wh = np.flip(img.shape[0:2])
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img = Image.fromarray(img)
    draw = ImageDraw.Draw(img)
    font = ImageFont.truetype(font='./YOLOKiwiFlower/data/fonts/futur.ttf',
                              size=(img.size[0] + img.size[1]) // 100)
    text_class = None
    diag_lenght = 0
    center_pos_box = (0, 0)
    colors2 = [(0,0,0), (250, 50, 0), (250, 50, 250), (0, 230, 230), (130,0,255), (255, 150, 0), (140, 0, 180), (0, 0, 150)]
    positions = pos_flowers.copy()
    if nums != len(positions):
        positions = [[[], [], []] for _ in range(nums)]

    for i in range(nums):  
        color = (0,0,0)
        x1y1 = ((np.array(boxes[i][0:2]) * wh).astype(np.int32))
        x2y2 = ((np.array(boxes[i][2:4]) * wh).astype(np.int32))
        thickness = (img.size[0] + img.size[1]) // 200
        x0, y0 = x1y1[0], x1y1[1]
        for t in np.linspace(0, 1, thickness):
            x1y1[0], x1y1[1] = x1y1[0] - t, x1y1[1] - t
            x2y2[0], x2y2[1] = x2y2[0] - t, x2y2[1] - t
            draw.rectangle([x1y1[0], x1y1[1], x2y2[0], x2y2[1]], outline=color)
        diag_lenght = distance_point(x1y1, x2y2)
        confidence = '{:.2f}%'.format(objectness[i]*100)
        text_class = class_names[int(classes[i])]
        text = '{} {}'.format(text_class, confidence)
        text_size = draw.textsize(text, font=font)
        draw.rectangle([x0 - 5, y0 - text_size[1] - 5, x0 + text_size[0], y0],
                        fill=color)
        draw.text((x0, y0 - text_size[1]), text, fill='white', font=font)
        center_pos_box = (x1y1[0] + int((x2y2[0] - x1y1[0]) / 2), x1y1[1] + int((x2y2[1] - x1y1[1]) / 2))
        draw.ellipse([(center_pos_box[0] - 2, center_pos_box[1] - 2), (center_pos_box[0] + 2, center_pos_box[1] + 2)], fill=colors2[i])
        
        # Find positions
        coord = flower_position(fig, center, center_pos_box, diag_lenght)
        positions[i][0].append(coord[0])
        positions[i][1].append(coord[1])
        positions[i][2].append(coord[2])
    
    rgb_img = img.convert('RGB')
    img_np = np.asarray(rgb_img)
    img = cv2.cvtColor(img_np, cv2.COLOR_BGR2RGB)

    return img, positions 


def draw_labels(x, y, class_names):
    colors = ((np.array(color_palette("hls", 80)) * 255)).astype(np.uint8)
    img = x.numpy()
    boxes, classes = tf.split(y, (4, 1), axis=-1)
    classes = classes[..., 0]
    wh = np.flip(img.shape[0:2])
    for i in range(len(boxes)):
        x1y1 = tuple((np.array(boxes[i][0:2]) * wh).astype(np.int32))
        x2y2 = tuple((np.array(boxes[i][2:4]) * wh).astype(np.int32))
        img = cv2.rectangle(img, x1y1, x2y2, (255, 0, 0), 2)
        img = cv2.putText(img, class_names[classes[i]],
                          x1y1, cv2.FONT_HERSHEY_COMPLEX_SMALL,
                          1, (0, 0, 0), 2)
    return img


def freeze_all(model, frozen=True):
    model.trainable = not frozen
    if isinstance(model, tf.keras.Model):
        for l in model.layers:
            freeze_all(l, frozen)
