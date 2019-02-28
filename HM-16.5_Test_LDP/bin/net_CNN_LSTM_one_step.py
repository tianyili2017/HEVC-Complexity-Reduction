#coding=utf-8

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
import tensorflow as tf
import config as cf

# external features, input to full connecting layers instead of convolutional layers
NUM_EXT_FEATURES_CNN = 1

NUM_CONVLAYER1_FILTERS = 16
NUM_CONVLAYER2_FILTERS = 24
NUM_CONVLAYER3_FILTERS = 32

NUM_CONV2_FLAT_S_FILTERS = 8 * 8 * NUM_CONVLAYER2_FILTERS
NUM_CONV2_FLAT_M_FILTERS = 4 * 4 * NUM_CONVLAYER2_FILTERS
NUM_CONV2_FLAT_L_FILTERS = 2 * 2 * NUM_CONVLAYER2_FILTERS

NUM_CONV3_FLAT_S_FILTERS = 4 * 4 * NUM_CONVLAYER3_FILTERS 
NUM_CONV3_FLAT_M_FILTERS = 2 * 2 * NUM_CONVLAYER3_FILTERS 
NUM_CONV3_FLAT_L_FILTERS = 1 * 1 * NUM_CONVLAYER3_FILTERS 

NUM_CONVLAYER_FLAT_FILTERS = NUM_CONV2_FLAT_S_FILTERS + NUM_CONV2_FLAT_M_FILTERS + NUM_CONV2_FLAT_L_FILTERS + NUM_CONV3_FLAT_S_FILTERS + NUM_CONV3_FLAT_M_FILTERS + NUM_CONV3_FLAT_L_FILTERS

NUM_DENLAYER1_FEATURES_64 = 64        
NUM_DENLAYER2_FEATURES_64 = 48

NUM_DENLAYER1_FEATURES_32 = 128       
NUM_DENLAYER2_FEATURES_32 = 96 

NUM_DENLAYER1_FEATURES_16 = 256       
NUM_DENLAYER2_FEATURES_16 = 192 

BATCH_SIZE = cf.BATCH_SIZE
IMAGE_SIZE = cf.IMAGE_SIZE
NUM_CHANNELS = cf.NUM_CHANNELS
GOP_LENGTH = cf.GOP_LENGTH
NUM_LABEL_BYTES = cf.NUM_LABEL_BYTES

VECTOR_LENGTH = cf.VECTOR_LENGTH
VECTOR_LENGTH_LIST = cf.VECTOR_LENGTH_LIST
LSTM_MAX_LENGTH = 1 
LSTM_READ_LENGTH = 1 
LSTM_OUTPUT_LENGTH = 1 
LSTM_DEPTH = 1

NUM_VECTOR_SIZE_64 = VECTOR_LENGTH_LIST[0]
NUM_VECTOR_SIZE_32 = VECTOR_LENGTH_LIST[1]
NUM_VECTOR_SIZE_16 = VECTOR_LENGTH_LIST[2]

NUM_HIDDEN_SIZE_64 = NUM_VECTOR_SIZE_64 
NUM_HIDDEN_SIZE_32 = NUM_VECTOR_SIZE_32 
NUM_HIDDEN_SIZE_16 = NUM_VECTOR_SIZE_16 
      
NUM_DENLAYER2_FEATURES_64 = 48
NUM_DENLAYER2_FEATURES_32 = 96       
NUM_DENLAYER2_FEATURES_16 = 192  

MAX_GRAD_NORM = 5
NUM_EXT_FEATURES = 5 # QP and i_frame_in_GOP (one hot)

def get_thresholds(thr_file):
    f = open(thr_file,'r+')
    line=f.readline()
    str_arr=line.split(' ')
    thr_l1_lower = float(str_arr[1])  
    thr_l2_lower = float(str_arr[3])
    f.close()
    return thr_l1_lower, thr_l2_lower

THR_L1_LOWER, THR_L2_LOWER = get_thresholds('Thr_info.txt')

# weight initialization
def weight_variable_cnn(shape, name=None):
    initial = tf.truncated_normal(shape, stddev=0.1)
    return tf.Variable(initial, name=name)

def bias_variable_cnn(shape, name=None):
    initial = tf.truncated_normal(shape, stddev=0.1)
    return tf.Variable(initial, name=name)

def weight_variable_lstm(shape, name=None, is_reuse_var=False):
    initial = tf.truncated_normal(shape, stddev=0.1)
    return tf.get_variable("full_connect_w", shape) # KEY POINT. To share variables, invoke "get_variable()" rather than "Variable()"

def bias_variable_lstm(shape, name=None, is_reuse_var=False):
    initial = tf.truncated_normal(shape, stddev=0.1)
    return tf.get_variable("full_connect_b", shape) # KEY POINT. To share variables, invoke "get_variable()" rather than "Variable()"	

# convolution
def conv2d(x, W):
    return tf.nn.conv2d(x, W, strides=[1, 1, 1, 1], padding='VALID')
# pooling
def max_pool_2x2(x):
    return tf.nn.max_pool(x, ksize=[1, 2, 2, 1], strides=[1, 2, 2, 1], padding='SAME')

def aver_pool(x, k_width):
    return tf.nn.avg_pool(x, ksize=[1, k_width, k_width, 1], strides=[1, k_width, k_width, 1], padding='SAME')

def activate(x, acti_mode, scope=None):
    if acti_mode==0:
        return x
    elif acti_mode==1:
        return tf.nn.leaky_relu(x)
    elif acti_mode==2:
        return tf.nn.sigmoid(x)  

def expand_dim(x, axis, num_dup):
    x_list=[]
    for i in range(num_dup):
        x_list.append(x)
    return tf.stack(x_list, axis)

def zero_mean_norm_global(x, reduce_axis=[1,2,3], tile_multiples=[1,64,64,1]):
    x_mean_reduced=tf.reduce_mean(x,axis=reduce_axis,keep_dims=True)
    x_mean_tiled=tf.tile(x_mean_reduced,tile_multiples)
    return x-x_mean_tiled

def zero_mean_norm_local(x, x_width, kernel_width):
    w_norm = tf.constant(1.0/(kernel_width*kernel_width), tf.float32, shape=[kernel_width, kernel_width,1,1])
    x_mean_reduced = tf.nn.conv2d(x, w_norm, [1, kernel_width, kernel_width, 1],'VALID')
    x_mean_expanded = tf.image.resize_nearest_neighbor(x_mean_reduced, [x_width, x_width])
    print(x_mean_reduced)
    print(x_mean_expanded)
    return x-x_mean_expanded

def non_overlap_conv(x, k_width, num_filters_in, num_filters_out, acti_mode):
    w_conv = weight_variable_cnn([k_width, k_width, num_filters_in, num_filters_out])
    b_conv = bias_variable_cnn([num_filters_out])
    h_conv = tf.nn.conv2d(x, w_conv, strides=[1, k_width, k_width, 1], padding='VALID') + b_conv
    h_conv = activate(h_conv, acti_mode)
    return(h_conv) 

def full_connect_cnn(x, num_filters_in, num_filters_out, acti_mode, keep_prob=1, name_w=None, name_b=None): 
    w_fc = weight_variable_cnn([num_filters_in, num_filters_out], name_w)
    b_fc = bias_variable_cnn([num_filters_out], name_b)
    h_fc = tf.matmul(x, w_fc) + b_fc
    h_fc = activate(h_fc, acti_mode)
    h_fc = tf.cond(keep_prob < tf.constant(1.0), lambda: tf.nn.dropout(h_fc, keep_prob), lambda: h_fc)
    return h_fc

def full_connect_lstm(x, num_filters_in, num_filters_out, acti_mode, keep_prob=1, name_w=None, name_b=None, is_reuse_var=False): 
    w_fc = weight_variable_lstm([num_filters_in, num_filters_out], name_w, is_reuse_var)
    b_fc = bias_variable_lstm([num_filters_out], name_b, is_reuse_var)
    h_fc = tf.matmul(x, w_fc) + b_fc
    h_fc = activate(h_fc, acti_mode)
    h_fc = tf.cond(keep_prob < tf.constant(1.0), lambda: tf.nn.dropout(h_fc, keep_prob), lambda: h_fc) 
    return h_fc

def resi_cnn(x):

    x = (x - 128) / 255.0 * 10
    x_image = tf.reshape(x, [-1, IMAGE_SIZE, IMAGE_SIZE, NUM_CHANNELS])
    #----------------------------------------------------------------------------------------------------------------------------------------------------------
    acti_mode_conv = 1
    # for extracting textures of 64*64 CTUs
    h_image_L = zero_mean_norm_local(aver_pool(x_image, 4), 16, 16)
    h_conv1_L = non_overlap_conv(h_image_L, 4, NUM_CHANNELS, NUM_CONVLAYER1_FILTERS, acti_mode=acti_mode_conv)
    h_conv2_L = non_overlap_conv(h_conv1_L, 2, NUM_CONVLAYER1_FILTERS, NUM_CONVLAYER2_FILTERS, acti_mode=acti_mode_conv)
    h_conv3_L = non_overlap_conv(h_conv2_L, 2, NUM_CONVLAYER2_FILTERS, NUM_CONVLAYER3_FILTERS, acti_mode=acti_mode_conv)

    # for extracting textures of 32*32 CTUs
    h_image_M = zero_mean_norm_local(aver_pool(x_image, 2), 32, 16)
    h_conv1_M = non_overlap_conv(h_image_M, 4, NUM_CHANNELS, NUM_CONVLAYER1_FILTERS, acti_mode=acti_mode_conv)
    h_conv2_M = non_overlap_conv(h_conv1_M, 2, NUM_CONVLAYER1_FILTERS, NUM_CONVLAYER2_FILTERS, acti_mode=acti_mode_conv)    
    h_conv3_M = non_overlap_conv(h_conv2_M, 2, NUM_CONVLAYER2_FILTERS, NUM_CONVLAYER3_FILTERS, acti_mode=acti_mode_conv)     

    # for extracting textures of 16*16 CTUs
    h_image_S = zero_mean_norm_local(x_image, 64, 16)
    h_conv1_S = non_overlap_conv(h_image_S, 4, NUM_CHANNELS, NUM_CONVLAYER1_FILTERS, acti_mode=acti_mode_conv)
    h_conv2_S = non_overlap_conv(h_conv1_S, 2, NUM_CONVLAYER1_FILTERS, NUM_CONVLAYER2_FILTERS, acti_mode=acti_mode_conv)  
    h_conv3_S = non_overlap_conv(h_conv2_S, 2, NUM_CONVLAYER2_FILTERS, NUM_CONVLAYER3_FILTERS, acti_mode=acti_mode_conv)      

    h_conv3_S_flat = tf.reshape(h_conv3_S, [-1, NUM_CONV3_FLAT_S_FILTERS])
    h_conv3_M_flat = tf.reshape(h_conv3_M, [-1, NUM_CONV3_FLAT_M_FILTERS])
    h_conv3_L_flat = tf.reshape(h_conv3_L, [-1, NUM_CONV3_FLAT_L_FILTERS])    
    h_conv2_S_flat = tf.reshape(h_conv2_S, [-1, NUM_CONV2_FLAT_S_FILTERS])
    h_conv2_M_flat = tf.reshape(h_conv2_M, [-1, NUM_CONV2_FLAT_M_FILTERS])
    h_conv2_L_flat = tf.reshape(h_conv2_L, [-1, NUM_CONV2_FLAT_L_FILTERS])

    h_conv_flat = tf.concat( values=[h_conv3_S_flat, h_conv3_M_flat, h_conv3_L_flat, h_conv2_S_flat, h_conv2_M_flat, h_conv2_L_flat], axis=1)

    
    #-------------------------------------------------------------------------------------------------------------------------------------------------------------
    acti_mode_fc = 1
    h_fc1_64 = full_connect_cnn(h_conv_flat, NUM_CONVLAYER_FLAT_FILTERS, NUM_DENLAYER1_FEATURES_64, 
                            acti_mode=acti_mode_fc, keep_prob=1, name_w='h_fc1__64__w', name_b='h_fc1__64__b')    

    h_fc1_32 = full_connect_cnn(h_conv_flat, NUM_CONVLAYER_FLAT_FILTERS, NUM_DENLAYER1_FEATURES_32, 
                            acti_mode=acti_mode_fc, keep_prob=1, name_w='h_fc1__32__w', name_b='h_fc1__32__b') 

    h_fc1_16 = full_connect_cnn(h_conv_flat, NUM_CONVLAYER_FLAT_FILTERS, NUM_DENLAYER1_FEATURES_16, 
                            acti_mode=acti_mode_fc, keep_prob=1, name_w='h_fc1__16__w', name_b='h_fc1__16__b')              

    vector = tf.concat([h_fc1_64, h_fc1_32, h_fc1_16], axis=1)
    vector = tf.reshape(vector, [-1, LSTM_MAX_LENGTH, VECTOR_LENGTH])
    opt_vars_all_CNN = [v for v in tf.trainable_variables()]
    return vector, opt_vars_all_CNN

def lstm(x, state_in_tensor, num_x_size_x, num_input_hidden_size, isdrop, 
        num_denlayers_2_features, num_denlayers_3_features, 
         qp_list, i_frame_in_GOP_one_hot_list, acti_mode_fc, is_keep_fc,
         variable_scope = 'RNN', name_string = '64'):
    def lstm_cell():
        return tf.contrib.rnn.LSTMCell(num_input_hidden_size, forget_bias = 1.0, cell_clip = 5.0, state_is_tuple=True)
    
    def attn_cell():
        return tf.contrib.rnn.DropoutWrapper(lstm_cell(), output_keep_prob = 1 - isdrop * 0.5)    
    
    cell = tf.contrib.rnn.MultiRNNCell([attn_cell() for _ in range(LSTM_DEPTH)], state_is_tuple=True)
    
    state_in = []
    for i_depth in range(LSTM_DEPTH):
        state_in_tuple_temp = tf.contrib.rnn.LSTMStateTuple(tf.reshape(state_in_tensor[:,i_depth,0,:],[-1,num_input_hidden_size]), tf.reshape(state_in_tensor[:,i_depth,1,:],[-1,num_input_hidden_size]))
        state_in.append(state_in_tuple_temp)
        print(state_in[i_depth])
    
    cell_inputs = tf.reshape(x, [-1, LSTM_READ_LENGTH, num_x_size_x])
    print(cell_inputs)
    
    cell_inputs = tf.split(cell_inputs, LSTM_READ_LENGTH, axis=1)
    for time_step in range(LSTM_READ_LENGTH):
        cell_inputs[time_step] = tf.reshape(cell_inputs[time_step], [-1, num_x_size_x])
    
    cell_inputs.reverse() 

    cell_outputs = []
    y_pred_flat_list = []
    h_fc3_line_list = []    
    state = state_in
    with tf.variable_scope(variable_scope):
        for time_step in range(LSTM_READ_LENGTH):
            if time_step > 0: 
                tf.get_variable_scope().reuse_variables()
            (cell_output_one_step, state) = cell(cell_inputs[time_step], state)
            cell_outputs.append(cell_output_one_step)    
            state_out = state
            if time_step < LSTM_OUTPUT_LENGTH:
                efs = tf.concat([qp_list[time_step], i_frame_in_GOP_one_hot_list[time_step]], axis = 1)
                with tf.variable_scope('fc2'):
                    h_fc1 = tf.concat([cell_outputs[time_step], efs], axis = 1)
                    h_fc2 = full_connect_lstm(h_fc1, num_input_hidden_size + GOP_LENGTH + 1, num_denlayers_2_features,
                                            acti_mode = acti_mode_fc, keep_prob = 1-isdrop*0.2, is_reuse_var=(time_step>0)) 
                with tf.variable_scope('fc3'):
                    h_fc2 = tf.concat([h_fc2, efs], axis = 1)
                    y_pred_flat_temp = full_connect_lstm(h_fc2, num_denlayers_2_features + GOP_LENGTH + 1, num_denlayers_3_features,
                                             acti_mode = 2, keep_prob = 1, is_reuse_var=(time_step>0)) 
                print(is_keep_fc[time_step])
                y_pred_flat = tf.cond(is_keep_fc[time_step], lambda: y_pred_flat_temp, lambda: tf.zeros([tf.shape(efs)[0], num_denlayers_3_features], tf.float32))
                y_pred_flat_list.append(y_pred_flat) 
    
    state_out_tensor_list=[]        
    for i_depth in range(LSTM_DEPTH):
        state_out_tensor_c = tf.reshape(state_out[i_depth][0],[-1,1,1,num_input_hidden_size])
        state_out_tensor_h = tf.reshape(state_out[i_depth][1],[-1,1,1,num_input_hidden_size])  
        state_out_tensor_curr_depth = tf.concat([state_out_tensor_c,state_out_tensor_h], axis=2)
        state_out_tensor_list.append(state_out_tensor_curr_depth)
    state_out_tensor=tf.concat(state_out_tensor_list, axis=1)
    
    y_pred_flat_list.reverse()
    h_fc3_line_list.reverse()

    return y_pred_flat_list, state_out_tensor

def net(x,state_in_total,y_,ef,isdrop,global_step,learning_rate_init,momentum,decay_step, decay_rate, partly_tuning_mode, limit_grad = False, is_balance = False):
    
    vector, opt_vars_all_CNN = resi_cnn(x) # image -> vector
    
    qp = tf.slice(ef, [0,0],[-1,LSTM_MAX_LENGTH])
    i_frame_in_GOP = tf.slice(ef, [0,LSTM_MAX_LENGTH],[-1,LSTM_MAX_LENGTH])
    i_frame_in_GOP_one_hot = tf.reshape(tf.one_hot(tf.to_int32(i_frame_in_GOP), depth = 4, axis = 2), [-1, LSTM_MAX_LENGTH, 4])
    
    state_in_tensor_64=tf.slice(state_in_total, [0,0,0,0],[-1,-1,-1,VECTOR_LENGTH_LIST[0]])
    state_in_tensor_32=tf.slice(state_in_total, [0,0,0,VECTOR_LENGTH_LIST[0]],[-1,-1,-1,VECTOR_LENGTH_LIST[1]])
    state_in_tensor_16=tf.slice(state_in_total, [0,0,0,VECTOR_LENGTH_LIST[0]+VECTOR_LENGTH_LIST[1]],[-1,-1,-1,VECTOR_LENGTH_LIST[2]])

    print(qp)
    print(i_frame_in_GOP_one_hot) 
    
    qp = qp / 51.0 * 0.18
    
    qp_list = []
    i_frame_in_GOP_one_hot_list = []
    
    for i in range(LSTM_OUTPUT_LENGTH):
        qp_list.append(tf.reshape(tf.slice(qp, [0, i], [-1, 1]), [-1, 1]))
        i_frame_in_GOP_one_hot_list.append(tf.reshape(tf.slice(i_frame_in_GOP_one_hot, [0, i, 0], [-1, 1, -1]),[-1, 4]))

    #----------------------------------------------------------------------------------------------------------------------------------------------------------
    # vector.shape = [-1, LSTM_MAX_LENGTH, VECTOR_LENGTH] = [-1, 20, 448]
    vector_64 = tf.slice(vector, [0, 0, 0], [-1, LSTM_READ_LENGTH, VECTOR_LENGTH_LIST[0]])
    vector_32 = tf.slice(vector, [0, 0, VECTOR_LENGTH_LIST[0]], [-1, LSTM_READ_LENGTH, VECTOR_LENGTH_LIST[1]])
    vector_16 = tf.slice(vector, [0, 0, VECTOR_LENGTH_LIST[0]+VECTOR_LENGTH_LIST[1]], [-1, LSTM_READ_LENGTH, VECTOR_LENGTH_LIST[2]])
    
    acti_mode_fc = 1
    
    y_pred_flat_64_list, state_out_tensor_64 = lstm(vector_64, state_in_tensor_64, NUM_VECTOR_SIZE_64, NUM_HIDDEN_SIZE_64, isdrop, 
         NUM_DENLAYER2_FEATURES_64, 1, 
         tf.multiply(1.0, qp_list), i_frame_in_GOP_one_hot_list, acti_mode_fc, [tf.cast(True, tf.bool) for i in range(LSTM_MAX_LENGTH)],
             variable_scope = 'RNN64', name_string ='64')   
    
    y_pred_flat_32_list, state_out_tensor_32 = lstm(vector_32, state_in_tensor_32, NUM_VECTOR_SIZE_32, NUM_HIDDEN_SIZE_32, isdrop, 
         NUM_DENLAYER2_FEATURES_32, 4, 
         tf.multiply(1.0, qp_list), i_frame_in_GOP_one_hot_list, acti_mode_fc, [tf.count_nonzero(y_pred_flat_64_list[i] > THR_L1_LOWER) > 0 for i in range(LSTM_MAX_LENGTH)],
             variable_scope = 'RNN32', name_string ='32')
    
    y_pred_flat_16_list, state_out_tensor_16 = lstm(vector_16, state_in_tensor_16, NUM_VECTOR_SIZE_16, NUM_HIDDEN_SIZE_16, isdrop, 
         NUM_DENLAYER2_FEATURES_16, 16, 
         tf.multiply(1.0, qp_list), i_frame_in_GOP_one_hot_list, acti_mode_fc, [tf.count_nonzero(y_pred_flat_32_list[i] > THR_L2_LOWER) > 0 for i in range(LSTM_MAX_LENGTH)],
             variable_scope = 'RNN16', name_string ='16')    
    
    state_out_total = tf.concat([state_out_tensor_64, state_out_tensor_32, state_out_tensor_16], axis=3)
    
    y_pred_flat_64 = tf.reshape(tf.concat(y_pred_flat_64_list, axis=1),[-1,1])
    y_pred_flat_32 = tf.reshape(tf.concat(y_pred_flat_32_list, axis=1),[-1,4])
    y_pred_flat_16 = tf.reshape(tf.concat(y_pred_flat_16_list, axis=1),[-1,16])
    
    #-----------------------------------------------------------------------------
    # variables in LSTM are all trainable variables except those in CNN
    opt_vars_all_LSTM = [v for v in tf.trainable_variables() if v not in opt_vars_all_CNN]
    
    return opt_vars_all_CNN, opt_vars_all_LSTM, y_pred_flat_64, y_pred_flat_32, y_pred_flat_16, state_out_total
