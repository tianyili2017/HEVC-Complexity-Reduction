#coding=utf-8
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import os,sys,shutil
import numpy as np
import random
import net_CNN as nt
import math
import tensorflow as tf
import time

os.environ['CUDA_VISIBLE_DEVICES'] = '' # only CPU is enabled

NUM_CHANNELS = nt.NUM_CHANNELS
NUM_EXT_FEATURES = nt.NUM_EXT_FEATURES
NUM_LABEL_BYTES = nt.NUM_LABEL_BYTES
IMAGE_SIZE = nt.IMAGE_SIZE
SAVE_FILE = 'cu_depth.dat'

x = tf.placeholder("float", [None, IMAGE_SIZE, IMAGE_SIZE, NUM_CHANNELS])
y_ = tf.placeholder("float", [None, NUM_LABEL_BYTES])
qp = tf.placeholder("float", [None, NUM_EXT_FEATURES])
isdrop = tf.placeholder("float")
y_flat_64, y_flat_32, y_flat_16, y_conv_flat_64, y_conv_flat_32, y_conv_flat_16, opt_vars_all = nt.net(x, y_, qp, isdrop) 

sess = tf.Session()
saver = tf.train.Saver(opt_vars_all, write_version = tf.train.SaverDef.V2)

def print_current_line(str):
    print('\r' + str, end = '')
    sys.stdout.flush()

def print_clear():
    print('\r', end = '')
    sys.stdout.flush()

def get_file_size(path):
    try:
        size = os.path.getsize(path)
        return size
    except Exception as err:
        print(err)

def get_Y_for_one_frame(f, frame_width, frame_height, image_size):
    y_buf = f.read(frame_width * frame_height)
    uv_temp = f.read(frame_width * frame_height//2)
    data = np.frombuffer(y_buf, dtype = np.uint8)
    data = data.reshape(1, frame_height * frame_width)
    valid_height = math.ceil(frame_height / image_size) * image_size
    valid_width = math.ceil(frame_width / image_size) * image_size
    data = data.reshape(frame_height, frame_width)
    if valid_height > frame_height:
        data = np.concatenate((data, np.zeros((valid_height - frame_height, frame_width))), axis = 0)
    if valid_width > frame_width:
        data = np.concatenate((data, np.zeros((valid_height, valid_width - frame_width))), axis = 1)
      
    return data
 
def get_y_conv_on_large_data(input_image, input_label, sess, qp_seq):
    batch_size = np.shape(input_image)[0]
    y_conv_out = np.zeros((batch_size, 21))
    sub_batch_size = 1024

    for i in range(math.ceil(batch_size / sub_batch_size)):
        index_start = i * sub_batch_size
        index_end = (i + 1) * sub_batch_size
        if index_end > batch_size:
            index_end = batch_size       
        y_flat_64_temp, y_flat_32_temp, y_flat_16_temp = sess.run([y_conv_flat_64, y_conv_flat_32, y_conv_flat_16], feed_dict={x: input_image[index_start:index_end, :].astype(np.float32), y_: input_label[index_start:index_end, :].astype(np.float32), qp: (np.ones((index_end - index_start, NUM_EXT_FEATURES)) * qp_seq).astype(np.float32), isdrop: 0})
        y_conv_out[index_start:index_end,:] = np.concatenate([y_flat_64_temp, y_flat_32_temp, y_flat_16_temp],axis=1)
    return y_conv_out

def get_prob(yuv_name, image_size, save_file, qp_seq, n_frames_start, n_frames_end, frame_width, frame_height):

    n_frames = n_frames_end - n_frames_start
    prob = np.zeros((n_frames * math.ceil(frame_height / image_size) * math.ceil(frame_width / image_size) , 21))
    f = open(yuv_name, 'rb')
    f_out = open(save_file, 'wb')
    
    valid_height = math.ceil(frame_height / image_size) * image_size
    valid_width = math.ceil(frame_width / image_size) * image_size
    
    for k in range(n_frames_start):
        luma = get_Y_for_one_frame(f, frame_width, frame_height, image_size)
    
    for k in range(n_frames):
        valid_luma = get_Y_for_one_frame(f, frame_width, frame_height, image_size)
        batch_size = (valid_height // image_size) * (valid_width // image_size)
        input_batch = np.zeros((batch_size, image_size, image_size, NUM_CHANNELS))
        input_label_temp = np.zeros((batch_size, NUM_LABEL_BYTES))
        
        index = 0
        ystart = 0
        while ystart < frame_height:
            xstart = 0
            while xstart < frame_width:
                CU_input = valid_luma[ystart : ystart + image_size, xstart : xstart + image_size]
                input_batch[index] = np.reshape(CU_input, [1, image_size, image_size, NUM_CHANNELS])
                input_label_temp[index] = np.zeros((1, NUM_LABEL_BYTES))
                index += 1
                xstart += image_size
            ystart += image_size  
            
        input_batch = input_batch.astype(np.float32)       
        
        y_conv_out = get_y_conv_on_large_data(input_batch, input_label_temp, sess, qp_seq)
        prob[k * batch_size : (k + 1) * batch_size] = y_conv_out 
        
        print_current_line('%s  frame %d/%d  %dx%d' % (yuv_name , k + 1, n_frames, frame_width, frame_height))
    
    print('')
    prob_arr = np.reshape(prob.astype(np.float32), [1, (n_frames * (valid_height // image_size) * (valid_width // image_size) * 21)])
    
    f_out.write(prob_arr) 
    f.close()
    f_out.close()    

assert len(sys.argv) == 5
yuv_file = sys.argv[1]
width = int(sys.argv[2])
height = int(sys.argv[3])
qp_seq = int(sys.argv[4])

if qp_seq < 25:
    saver.restore(sess, 'model_2000000_qp20~25.dat')
elif qp_seq < 30:
    saver.restore(sess, 'model_2000000_qp25~30.dat')
elif qp_seq < 35:
    saver.restore(sess, 'model_2000000_qp30~35.dat')
else:
    saver.restore(sess, 'model_2000000_qp35~40.dat')

file_bytes = get_file_size(yuv_file)
frame_bytes = width * height * 3 // 2
assert(file_bytes % frame_bytes == 0)

n_frames_start = 0
n_frames_end = file_bytes // frame_bytes

t1 = time.time()
get_prob(yuv_file, IMAGE_SIZE, SAVE_FILE, qp_seq, n_frames_start, n_frames_end, width, height)
t2 = time.time()
print('--------\n\nPredicting Time: %.3f sec.\n\n--------' % float(t2-t1))

