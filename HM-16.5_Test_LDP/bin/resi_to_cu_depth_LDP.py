#coding=utf-8
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import os,sys,shutil

import numpy as np
import random
import config as cf
import net_CNN_LSTM_one_step as nt
import math
import tensorflow as tf

os.environ['CUDA_VISIBLE_DEVICES'] = '' # only CPU is enabled

IMAGE_SIZE = cf.IMAGE_SIZE
NUM_CHANNELS = cf.NUM_CHANNELS
NUM_EXT_FEATURES = 2 # QP and POC
NUM_LABEL_BYTES = cf.NUM_LABEL_BYTES

VECTOR_LENGTH_LIST = cf.VECTOR_LENGTH_LIST
VECTOR_LENGTH = cf.VECTOR_LENGTH

LSTM_MAX_LENGTH = 1 # one-step LSTM
LSTM_DEPTH = 1

x = tf.placeholder("float", [None, IMAGE_SIZE, IMAGE_SIZE, NUM_CHANNELS])
y_ = tf.placeholder("float", [None, LSTM_MAX_LENGTH, NUM_LABEL_BYTES])
ef = tf.placeholder("float", [None, NUM_EXT_FEATURES])
state_in_total = tf.placeholder("float", [None, LSTM_DEPTH, 2, VECTOR_LENGTH]) # 4 dims: batch size, LSTM depth, c and h, vector_length
isdrop = tf.placeholder("float")
global_step = tf.placeholder("float")

opt_vars_all_CNN, opt_vars_all_LSTM, y_pred_flat_64, y_pred_flat_32, y_pred_flat_16, state_out_total = nt.net(x,state_in_total,y_,ef,isdrop,global_step,0.1,0.9,10000, 0.1, 0)

sess = tf.Session()

saver_CNN = tf.train.Saver(opt_vars_all_CNN, write_version=tf.train.SaverDef.V2)
saver_LSTM = tf.train.Saver(opt_vars_all_LSTM, write_version=tf.train.SaverDef.V2)

def send_init_signal(init_file):
    f=open(init_file,'w+')
    f.write('1')
    f.close()
    print('Python: Tensorflow initialized.')
    
def send_complete_signal(complete_file):
    f=open(complete_file,'w+')
    f.write('1')
    f.close()
    print('Python: Operation completed.')      

def get_command(command_file):
    f=open(command_file,'r+')
    line=f.readline()
    str_arr=line.split(' ')
    if len(str_arr)==5 and str_arr[4]=='[end]':
        print(str_arr[0], str_arr[1], str_arr[2], str_arr[3])
        i_frame = int(str_arr[0])
        frame_width = int(str_arr[1])
        frame_height = int(str_arr[2]) 
        qp_seq = int(str_arr[3])
    else:
        i_frame = -1
        frame_width = -1
        frame_height = -1
        qp_seq = -1      
    f.close()
    return i_frame, frame_width, frame_height, qp_seq

def get_images_from_one_file(yuv_file, frame_width, frame_height, CUwidth):
    f=open(yuv_file,'rb')
    y_buf=f.read(frame_width*frame_height)
    uv_temp=f.read(frame_width*frame_height//2)
    f.close()
    data = np.frombuffer(y_buf, dtype=np.uint8)
    data = data.reshape(1, frame_height*frame_width)
    valid_height = math.ceil(frame_height/CUwidth) * CUwidth
    valid_width = math.ceil(frame_width/CUwidth) * CUwidth
    data = data.reshape(frame_height, frame_width)
    if valid_height > frame_height:
        data=np.concatenate((data,np.zeros((valid_height-frame_height,frame_width))),axis=0)
    if valid_width > frame_width:
        data=np.concatenate((data,np.zeros((valid_height, valid_width-frame_width))),axis=1)
    
    num_vectors=(valid_height//CUwidth)*(valid_width//CUwidth)
    images=np.zeros((num_vectors,CUwidth,CUwidth,NUM_CHANNELS))
    
    index=0
    ystart=0
    while ystart < frame_height:
        xstart=0
        while xstart < frame_width:
            CU_input=data[ystart:ystart+CUwidth,xstart:xstart+CUwidth]
            images[index]=np.reshape(CU_input,[1,CUwidth,CUwidth,NUM_CHANNELS])
            index+=1
            xstart+=CUwidth
        ystart+=CUwidth
    images = images.astype(np.float32)  
    return images, num_vectors

def get_state_in_from_one_file(file_name, num_vectors, i_frame):
    if i_frame > 1:
        f_in=open(state_file,'rb')
        state_in_buf=f_in.read(num_vectors * LSTM_DEPTH * 2 * VECTOR_LENGTH * 4)
        state_in=np.frombuffer(state_in_buf, dtype=np.float32) 
        state_in = np.reshape(state_in, [num_vectors, LSTM_DEPTH, 2, VECTOR_LENGTH])
        f_in.close()    
    else:
        state_in = np.zeros((num_vectors, LSTM_DEPTH, 2, VECTOR_LENGTH))    
    return state_in

def predict_cu_depth(images,state_in,qp_seq,i_frame):
    batch_size=np.shape(images)[0]
    labels_virtual=np.zeros((batch_size, 1, NUM_LABEL_BYTES))
    efs=np.zeros((batch_size, 2))
    efs[:,0]=qp_seq
    efs[:,1]=i_frame%4
    depth_out=np.zeros((batch_size, 1 + 4 + 16))
    state_out_total_value = np.zeros((batch_size, LSTM_DEPTH, 2, VECTOR_LENGTH))
    mini_batch_size = 1024
    for i in range(math.ceil(batch_size / mini_batch_size)):
        index_start = i * mini_batch_size
        index_end = min([batch_size, (i + 1) * mini_batch_size])
        y_pred_flat_64_temp, y_pred_flat_32_temp, y_pred_flat_16_temp, state_out_total_temp = sess.run([y_pred_flat_64, y_pred_flat_32, y_pred_flat_16, state_out_total],feed_dict={x:images[index_start:index_end,:], y_:labels_virtual[index_start:index_end,:], ef:efs[index_start:index_end,:], isdrop:0, global_step:0, state_in_total:state_in[index_start:index_end,:]})
        depth_out[index_start:index_end,:] = np.concatenate([y_pred_flat_64_temp,y_pred_flat_32_temp,y_pred_flat_16_temp], axis = 1)  
        state_out_total_value[index_start:index_end,:] = state_out_total_temp  
    return depth_out, state_out_total_value

def save_cu_depth_and_state(depth_out,state_out,save_file,state_file,end_file,num_vectors):
    depth_out_line= np.reshape(depth_out.astype(np.float32),[1, num_vectors*(1+4+16)])
    state_out_line = np.reshape(state_out.astype(np.float32),[1, num_vectors*LSTM_DEPTH*2*VECTOR_LENGTH])
    
    print(save_file)
    f_out=open(state_file,'wb')
    f_out.write(state_out_line)
    f_out.close()
    f_out=open(save_file,'wb')
    f_out.write(depth_out_line)
    f_out.close()    
    
    f_out=open(end_file,'wb') # generate ending signal
    f_out.close()

if __name__ == "__main__":
    n_frame_total = 0
    qp_seq = 0
    
    complete_file='complete.dat'
    yuv_file='resi.yuv'
    state_file='state.dat'
    save_file='cu_depth.dat'
    command_file='command.dat'
    start_file='pred_start.sig'
    end_file='pred_end.sig'
    
    model_CNN_file='model_LDP_2000000_qp22~37.dat'        
    saver_CNN.restore(sess, model_CNN_file)
    print('Python: Tensorflow initialized.')
    
    while True:
        if os.path.isfile(start_file):
            i_frame, frame_width, frame_height, qp_seq_temp = get_command(command_file)
            if i_frame>=0:
                qp_seq_last = qp_seq
                qp_seq = qp_seq_temp
                os.remove(start_file)
                if(qp_seq!=qp_seq_last):
                    if qp_seq<25:
                        model_LSTM_file='model_LDP_200000_qp22.dat'  
                    elif qp_seq<30:
                        model_LSTM_file='model_LDP_200000_qp27.dat'  
                    elif qp_seq<35:
                        model_LSTM_file='model_LDP_200000_qp32.dat' 
                    else:
                        model_LSTM_file='model_LDP_200000_qp37.dat'                     
                    saver_LSTM.restore(sess, model_LSTM_file)  
                    print('Set QP = %d' % qp_seq)
                    print('LSTM model loaded.')
                
                images, num_vectors = get_images_from_one_file(yuv_file, frame_width, frame_height, 64)
                state_in = get_state_in_from_one_file(state_file, num_vectors, i_frame)
                depth_out, state_out = predict_cu_depth(images,state_in,qp_seq, i_frame)
                save_cu_depth_and_state(depth_out,state_out,save_file,state_file,end_file,num_vectors)
                n_frame_total += 1
                print('%d frames predicted.'% n_frame_total)


