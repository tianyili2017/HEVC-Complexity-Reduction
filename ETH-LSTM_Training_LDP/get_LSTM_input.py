import os,sys,shutil
import numpy as np
import matplotlib.pyplot as plt 
from matplotlib.pyplot import plot
from matplotlib.patches import Rectangle
import random
import net_CTU64_resi_CNN as nt
import math
import file_reader as fr
import config as cf
import tensorflow as tf

# ----------------------------------------------------------------------------------------------------------------------
# To configure: three variables (MODEL_FILE, INPUT_PATH, OUTPUT_PATH). 
# Other two variables (INPUT_NAME_LIST, OUTPUT_NAME_ROOT_LIST) need not be modified, unless you prefer to use other names.
MODEL_FILE = '/media/D/HEVC-Complexity-Reduction/ETH-CNN_Training_LDP/Models/model_20190306_215602_1000000_qp22~37.dat'
INPUT_PATH = '/media/F/Data/'
INPUT_NAME_LIST = ['LDP_Train_9011161.dat', 'LDP_Valid_1057660.dat', 'LDP_Test_1896004.dat']
OUTPUT_PATH = '/media/F/Data/'
OUTPUT_NAME_ROOT_LIST = ['LDP_Train', 'LDP_Valid', 'LDP_Test']
# ----------------------------------------------------------------------------------------------------------------------
# After this program completes, 6 files will be generated:
# (1) LDP_Test_716632.dat_lstm_4qps
# (2) LDP_Train_3429356.dat_lstm_4qps
# (3) LDP_Valid_403000.dat_lstm_4qps
# (4) LDP_Test_716632.dat_lstm_4qps_shuffled
# (5) LDP_Train_3429356.dat_lstm_4qps_shuffled
# (6) LDP_Valid_403000.dat_lstm_4qps_shuffled
# The combination of (1)~(3) or (4)~(6) can be used to train ETH-LSTM.
# Because (4)~(6) are shuffled version, they tend to provide better training performance than (1)~(3).
# So, after obtaining (4)~(6), (1)~(3) can be safely deleted to save the disk space.
# ----------------------------------------------------------------------------------------------------------------------

config = tf.ConfigProto()
config.gpu_options.per_process_gpu_memory_fraction = 0.2
sess=tf.Session(config = config)

QP_LIST = [22, 27, 32, 37]
NUM_QPS = len(QP_LIST)

np.set_printoptions(threshold=np.NaN)

IMAGE_SIZE = cf.IMAGE_SIZE
NUM_CHANNELS = cf.NUM_CHANNELS
NUM_EXT_FEATURES_CNN = cf.NUM_EXT_FEATURES_CNN
NUM_LABEL_BYTES = cf.NUM_LABEL_BYTES

VECTOR_LENGTH = cf.VECTOR_LENGTH
LSTM_MAX_LENGTH = cf.LSTM_MAX_LENGTH
LSTM_OVERLAP_STRIDE = cf.LSTM_OVERLAP_STRIDE

NUM_SAMPLE_LENGTH_PER_QP = 1 + NUM_LABEL_BYTES + IMAGE_SIZE * IMAGE_SIZE
NUM_SAMPLE_LENGTH = 64 + NUM_SAMPLE_LENGTH_PER_QP * NUM_QPS

# Sample bytes before transfering = 64+(1+16+4096)*4 = 16516
# Sample bytes before transfering = 64+(1+16+448)*4*20 = 37264 (for single QP)

LABEL_LENGTH = NUM_QPS * (1 + NUM_LABEL_BYTES) 

BATCH_SAMPLES = 20000
BATCH_SAMPLES_SWAP = 5000

NUM_SAMPLE_LENGTH_OUT = 64 + (1 + NUM_LABEL_BYTES + VECTOR_LENGTH) * 4 * LSTM_MAX_LENGTH

def get_file_size(path):
    try:
        size = os.path.getsize(path)
        return size
    except Exception as err:
        print(err)

def get_delta_ref_frames(i_frame):        
    if i_frame>=LSTM_MAX_LENGTH-1:
        return list(range(-1, -LSTM_MAX_LENGTH, -1))
    else:
        return list(range(-1, -(i_frame+1), -1))
    
def get_is_print(i_frame):
    if i_frame % LSTM_OVERLAP_STRIDE == 0:
        return True
    else:
        return False
        
def get_vectors(images):
# Feed image matrix (uint8) into ETH-CNN，to obtain the three vectors (float32) after the 1st full-connected layer
    num_samples=images.shape[0]
    sub_batch_size=100
    vectors = np.zeros((num_samples, VECTOR_LENGTH)).astype(np.float32)
    for i in range(math.ceil(num_samples/sub_batch_size)):
        index_start=i*sub_batch_size
        index_end=(i+1)*sub_batch_size
        if index_end>num_samples:
            index_end=num_samples
        labels_virtual=np.zeros((index_end-index_start, NUM_LABEL_BYTES))
        qps_virtual=np.ones((index_end-index_start, NUM_EXT_FEATURES_CNN))        
        y_conv_fc_64_temp, y_conv_fc_32_temp, y_conv_fc_16_temp = sess.run([y_conv_fc_64, y_conv_fc_32, y_conv_fc_16],feed_dict={x:images[index_start:index_end,:,:,:].astype(np.float32), y_:labels_virtual[index_start:index_end,:], qp:qps_virtual[index_start:index_end,:], isdrop:0, global_step:0})
        vectors[index_start:index_end,:]=np.concatenate([y_conv_fc_64_temp,y_conv_fc_32_temp,y_conv_fc_16_temp],axis=1)
    return vectors

def get_labels_part_qps(labels_allqps, num_samples):
    labels_part_qps = np.zeros((num_samples, (NUM_LABEL_BYTES+1)*NUM_QPS)).astype(np.uint8) 
    for i_qp in range(NUM_QPS):
        labels_part_qps[:,(NUM_LABEL_BYTES+1)*i_qp] = QP_LIST[i_qp]
        labels_part_qps[:,(NUM_LABEL_BYTES+1)*i_qp+1:(NUM_LABEL_BYTES+1)*(i_qp+1)] = labels_allqps[:,QP_LIST[i_qp]*NUM_LABEL_BYTES:(QP_LIST[i_qp]+1)*NUM_LABEL_BYTES]  
    return labels_part_qps

def shuffle_samples(file, sample_length):
    file_bytes = get_file_size(file)
    assert(file_bytes % sample_length == 0)
    num_samples = file_bytes // sample_length
    index_list = random.sample(range(num_samples), num_samples)
    fid_in = open(file, 'rb')
    fid_out = open(file + '_shuffled', 'wb')
    for i in range(num_samples):
        fid_in.seek(index_list[i] * sample_length, 0)
        info_buf = fid_in.read(sample_length)
        fid_out.write(info_buf)
        if (i + 1) % 100 == 0:
            print('%s : %d / %d samples completed.' % (file, i + 1, num_samples))
    fid_in.close()
    fid_out.close()
    
x = tf.placeholder("float", [None, IMAGE_SIZE, IMAGE_SIZE, NUM_CHANNELS])
y_ = tf.placeholder("float", [None, NUM_LABEL_BYTES])
qp = tf.placeholder("float", [None, NUM_EXT_FEATURES_CNN])
isdrop = tf.placeholder("float")
global_step = tf.placeholder("float")

y_flat_64, y_flat_32, y_flat_16, y_conv_flat_64, y_conv_flat_32, y_conv_flat_16, total_loss, loss_list, learning_rate_current, train_step, accuracy_list, opt_vars_all, y_conv_fc_64, y_conv_fc_32, y_conv_fc_16 = nt.net(x,y_,qp,isdrop,global_step,0.1, 0.9,10000,0.1, 0)

saver = tf.train.Saver(opt_vars_all, write_version = tf.train.SaverDef.V2)
saver.restore(sess, MODEL_FILE)

def get_one_data_file(input_file, output_file_root):

    read_index_list = [[0, get_file_size(input_file) // NUM_SAMPLE_LENGTH]]

    read_index_list_arr = np.array(read_index_list)
    num_samples_total = np.sum(read_index_list_arr[:,1]-read_index_list_arr[:,0])

    print(num_samples_total)

    file_reader = fr.FileReader()
    file_reader.initialize(input_file, get_file_size(input_file))
    output_file_temp = output_file_root + '.dat_shuffled'
    f_out = open(output_file_temp, 'wb')

    num_samples_saved = 0
    num_samples_saved_valid = 0

    for i_qp in range(NUM_QPS):
        for i in range(len(read_index_list)):
            if i==0:
                index_last = 0
            else:
                index_last = read_index_list[i-1][1]

            index_start = read_index_list[i][0]
            index_end = read_index_list[i][1]
            assert index_end > index_start
            assert index_start >= index_last
            num_samples = index_end - index_start

            if index_start > index_last:
                file_reader.read_data((index_start - index_last)*NUM_SAMPLE_LENGTH, isloop=False, dtype=np.uint8)

            index_start_in_batch = 0
            index_end_in_batch = 0
            vectors_lstm = []
            is_init = True
            while index_end_in_batch < num_samples:
                if is_init == True:
                    index_start_in_batch = 0
                    index_new_start_in_batch = 0
                    index_end_in_batch = BATCH_SAMPLES
                else:
                    index_start_in_batch += BATCH_SAMPLES_SWAP
                    index_new_start_in_batch = index_end_in_batch
                    index_end_in_batch += BATCH_SAMPLES_SWAP
                if index_end_in_batch > num_samples:
                    index_end_in_batch = num_samples

                num_samples_new = index_end_in_batch-index_new_start_in_batch

                data_new = file_reader.read_data(num_samples_new * NUM_SAMPLE_LENGTH, isloop=False, dtype=np.uint8)
                data_new = data_new.reshape(num_samples_new, NUM_SAMPLE_LENGTH)

                data_info_new = np.copy(data_new[:, 0:64]) 
                data_new = np.copy(data_new[:, 64 + NUM_SAMPLE_LENGTH_PER_QP * i_qp : 64 + NUM_SAMPLE_LENGTH_PER_QP * (i_qp+1)])

                images_new = np.copy(data_new[:,1+NUM_LABEL_BYTES:]) 
                images_new = np.reshape(images_new,[num_samples_new, IMAGE_SIZE, IMAGE_SIZE, NUM_CHANNELS]) 

                vectors_lstm_new = 255*np.ones((num_samples_new, VECTOR_LENGTH * LSTM_MAX_LENGTH)).astype(np.float32) 
                vectors_lstm_new[:,0:VECTOR_LENGTH] = get_vectors(images_new)
                labels_qps_new = 255*np.ones((num_samples_new, (1 + NUM_LABEL_BYTES) * LSTM_MAX_LENGTH)).astype(np.uint8) 
                labels_qps_new[:,0:1+NUM_LABEL_BYTES] = data_new[:, 0:1+NUM_LABEL_BYTES]

                if is_init==True :
                    vectors_lstm = vectors_lstm_new
                    labels_qps = labels_qps_new
                else:
                    vectors_lstm = np.concatenate([vectors_lstm[BATCH_SAMPLES_SWAP: vectors_lstm.shape[0]], vectors_lstm_new], axis=0)
                    labels_qps = np.concatenate([labels_qps[BATCH_SAMPLES_SWAP: labels_qps.shape[0]], labels_qps_new], axis=0)

                widths_new = data_info_new[:,2] + 256 * data_info_new[:,3] 
                heights_new = data_info_new[:,4] + 256 * data_info_new[:,5] 
                widths_in_64_new = widths_new // 64 
                heights_in_64_new = heights_new // 64 

                i_frames_new = data_info_new[:,10] + 256 * data_info_new[:,11] + 256 * 256 * data_info_new[:,12] + 256 * 256 * 256 * data_info_new[:,13]

                num_samples_valid_currentbatch = 0
                for i_sample_new in range(num_samples_new):
                    delta_frames_new=get_delta_ref_frames(i_frames_new[i_sample_new]) 
                    data_info_new[i_sample_new,0]=len(delta_frames_new) 

                    if len(delta_frames_new)+1 == LSTM_MAX_LENGTH and get_is_print(i_frames_new[i_sample_new]) == True:
                        num_samples_saved_valid += 1
                        num_samples_valid_currentbatch += 1

                        f_out.write(data_info_new[i_sample_new,:])
                        f_out.write(labels_qps_new[i_sample_new, 0:(1+NUM_LABEL_BYTES)].astype(np.float32))
                        f_out.write(vectors_lstm_new[i_sample_new, 0:VECTOR_LENGTH].astype(np.float32))

                        for i_delta_frame in range(len(delta_frames_new)):
                            i_sample_ref_new=i_sample_new-i_delta_frame*widths_in_64_new[i_sample_new]*heights_in_64_new[i_sample_new] 
                            if is_init==True:
                                i_sample_ref_total = i_sample_ref_new
                            else:
                                i_sample_ref_total = i_sample_ref_new + BATCH_SAMPLES - BATCH_SAMPLES_SWAP
                            
                            if i_sample_ref_total>=0:
                                vectors_lstm_new[i_sample_new, VECTOR_LENGTH*(i_delta_frame+1):VECTOR_LENGTH*(i_delta_frame+2)] = vectors_lstm[i_sample_ref_total, 0:VECTOR_LENGTH]
                                labels_qps_new[i_sample_new, (1+NUM_LABEL_BYTES)*(i_delta_frame+1):(1+NUM_LABEL_BYTES)*(i_delta_frame+2)] = labels_qps[i_sample_ref_total, 0:(1+NUM_LABEL_BYTES)]

                            f_out.write(labels_qps[i_sample_ref_total, 0:(1+NUM_LABEL_BYTES)].astype(np.float32))
                            f_out.write(vectors_lstm[i_sample_ref_total, 0:VECTOR_LENGTH].astype(np.float32))

                is_init = False
                num_samples_saved += num_samples_new
                print('QP No.%d, %d (%d Valid) / %d Samples Completed' % (i_qp+1, num_samples_saved, num_samples_saved_valid, num_samples_total * NUM_QPS))
    f_out.close()

    output_file = output_file_root + ('_%d.dat_lstm_%dqps' % (num_samples_saved_valid, NUM_QPS))
    os.rename(output_file_temp, output_file)
    shuffle_samples(output_file, NUM_SAMPLE_LENGTH_OUT * NUM_QPS)

if __name__ == '__main__':
    for i_file in range(len(INPUT_NAME_LIST)):
        get_one_data_file(
            os.path.join(INPUT_PATH, INPUT_NAME_LIST[i_file]),
            os.path.join(OUTPUT_PATH, OUTPUT_NAME_ROOT_LIST[i_file]))





