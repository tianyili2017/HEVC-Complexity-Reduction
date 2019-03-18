import net_CTU64 as nt 
import file_reader as fr
import random
import os
import numpy as np
import config as cf

# Each sample contains data and labels for only one QP
# Format: [info] [[QP] [labels] [vector]]*20
# 64+(1+16+VECTOR_LENGTH)*LSTM_MAX_LENGTH

DATA_PATH = '/media/F/Data/'

NUM_EXT_FEATURES = cf.NUM_EXT_FEATURES
NUM_LABEL_BYTES = cf.NUM_LABEL_BYTES
VECTOR_LENGTH = cf.VECTOR_LENGTH 
LSTM_MAX_LENGTH = cf.LSTM_MAX_LENGTH
GOP_LENGTH = cf.GOP_LENGTH

DEFAULT_THR_LIST = [0.5, 1.5, 2.5]

TRAINSET_MAXSIZE = 10000000
VALIDSET_MAXSIZE = 10000000
TESTSET_MAXSIZE = 10000000

TRAINSET_READSIZE = 25000
VALIDSET_READSIZE = 10000
TESTSET_READSIZE = 10000

TRAIN_FILE_READER = []
VALID_FILE_READER = []
TEST_FILE_READER = []

TRAINSET = []
VALIDSET = []
TESTSET  = []

IS_SELECT_QP = False
SELECT_QP_LIST = []

MODEL_TYPE = 1 # switch from 1~4
if MODEL_TYPE == 1:
    MODEL_NAME='qp22'   
    IS_SELECT_QP = True
    SELECT_QP_LIST=[22]
    EVALUATE_QP_THR_LIST = [25,30,35]
elif MODEL_TYPE == 2:
    MODEL_NAME='qp27'   
    IS_SELECT_QP = True
    SELECT_QP_LIST=[27]
    EVALUATE_QP_THR_LIST = [25,30,35]
elif MODEL_TYPE == 3:
    MODEL_NAME='qp32'   
    IS_SELECT_QP = True
    SELECT_QP_LIST=[32]
    EVALUATE_QP_THR_LIST = [25,30,35]
elif MODEL_TYPE == 4:
    MODEL_NAME='qp37'   
    IS_SELECT_QP = True
    SELECT_QP_LIST=[37]
    EVALUATE_QP_THR_LIST = [25,30,35]

DATA_SWITCH = 0
# above are global variables ------------------------------------------

def get_train_valid_test_sets(DATA_SWITCH):
    global NUM_QPS, NUM_SAMPLE_LENGTH, TRAINSET, VALIDSET, TESTSET
    if DATA_SWITCH == 0:
        TRAINSET = 'LDP_Train_3429356.dat_lstm_4qps_shuffled'
        VALIDSET = 'LDP_Valid_403000.dat_lstm_4qps_shuffled'
        TESTSET = 'LDP_Test_716632.dat_lstm_4qps_shuffled'
    if DATA_SWITCH == 1: # switch to other files
        TRAINSET = '...'
        VALIDSET = '...'
        TESTSET  = '...'
        
    TRAINSET = DATA_PATH + TRAINSET
    VALIDSET = DATA_PATH + VALIDSET
    TESTSET  = DATA_PATH + TESTSET

    filename_end = str.split(TRAINSET, '_')[-1]
    if filename_end == 'shuffled':
        filename_end = str.split(TRAINSET, '_')[-2]
    NUM_QPS = int(filename_end[:-3])

    NUM_SAMPLE_LENGTH = VECTOR_LENGTH * LSTM_MAX_LENGTH * 4 + 64 + (NUM_LABEL_BYTES+1) * NUM_QPS * LSTM_MAX_LENGTH 

def get_delta_ref_frames(i_frame):      
    if i_frame>=LSTM_MAX_LENGTH-1:
        return list(range(-1, -LSTM_MAX_LENGTH, -1))
    else:
        return list(range(-1, -(i_frame+1), -1))

def get_data_set(file_reader, num_samples, is_loop=True):
    
    vectors = np.zeros((num_samples, LSTM_MAX_LENGTH, VECTOR_LENGTH)).astype(np.float32)
    data_info = np.zeros((num_samples, 64)).astype(np.uint8)
    
    qps = np.zeros((num_samples, LSTM_MAX_LENGTH)).astype(np.uint8) 
    labels = np.zeros((num_samples, LSTM_MAX_LENGTH, NUM_LABEL_BYTES)).astype(np.uint8)
    
    for i in range(num_samples):
        data_info[i,:] = np.reshape(file_reader.read_data(64, isloop=is_loop, dtype=np.uint8),[1, 64])      
        qps_labels_vectors_temp = file_reader.read_data(4 * (1 + NUM_LABEL_BYTES + VECTOR_LENGTH) * LSTM_MAX_LENGTH, isloop=is_loop, dtype=np.float32)
        qps_labels_vectors_temp = np.reshape(qps_labels_vectors_temp,[LSTM_MAX_LENGTH, 1 + NUM_LABEL_BYTES + VECTOR_LENGTH])
        qps[i,:]=np.reshape(qps_labels_vectors_temp[:,0],[1,LSTM_MAX_LENGTH]).astype(np.uint8)
        labels[i,:,:]=np.reshape(qps_labels_vectors_temp[:,1:1+NUM_LABEL_BYTES], [1, LSTM_MAX_LENGTH, NUM_LABEL_BYTES]).astype(np.uint8)
        vectors[i,:,:]=np.reshape(qps_labels_vectors_temp[:,1+NUM_LABEL_BYTES:], [1, LSTM_MAX_LENGTH, VECTOR_LENGTH]).astype(np.float32)
    
    lstm_length = data_info[:, 0].reshape(num_samples, 1).astype(np.uint8)
    
    i_frame_base_address = 10
    i_frame_one_frame = data_info[:, i_frame_base_address] + 256 * data_info[:, i_frame_base_address+1] + 256*256*data_info[:, i_frame_base_address+2] + 256*256*256*data_info[:, i_frame_base_address+3] 
    i_frame = np.zeros((num_samples, LSTM_MAX_LENGTH)).astype(np.int)
    
    i_frame[:, 0] = i_frame_one_frame
    for i in range(num_samples): 
        i_frame[i,1:] = i_frame_one_frame[i] + get_delta_ref_frames(i_frame_one_frame[i])
  
    i_frame_in_GOP = np.mod(i_frame, GOP_LENGTH).astype(np.uint8)
    
    efs = np.concatenate([lstm_length, qps, i_frame_in_GOP], axis=1)
    
    assert efs.shape[1] == NUM_EXT_FEATURES
    
    i_select_list=[]
    if IS_SELECT_QP == True:
        for i in range(num_samples):
            if qps[i,0] in SELECT_QP_LIST:
                i_select_list.append(i)
        vectors_select = vectors[i_select_list, :]
        labels_select = labels[i_select_list, :] 
        efs_select = efs[i_select_list, :] 
    else:
        vectors_select = vectors
        labels_select = labels
        efs_select = efs        
        
    range_stat = RangingStatistics(DEFAULT_THR_LIST, 'scalar')
    count_list_ori, _ = range_stat.feed_data_list(labels)
    print(count_list_ori, end='  ')
    if IS_SELECT_QP == True:
        range_stat.clear_count()
        count_list_select, _ = range_stat.feed_data_list(labels_select)
        print(count_list_select)              
        
    return DataSet(vectors_select, labels_select, efs_select)  

class RangingStatistics(object):
    def __init__(self, thr_list, formula):
        self._thr_list=thr_list
        self._formula=formula
        self._count_list=np.zeros((len(thr_list)+1))
    
    @property
    def get_thr_list(self):
        return self._thr_list
    
    @property
    def get_formula(self):
        return self._formula
    
    @property
    def get_count_list(self):
        return self._count_list 
    
    def clear_count(self):
        self._count_list=np.zeros((len(self._thr_list)+1))
        
    def set_thr_list(self,thr_list):
        self._thr_list=thr_list
        self._count_list=np.zeros((len(thr_list)+1))
        
    def get_segment_names(self, variable_name):
        names=list(range(len(self._count_list)))
        for i in range(len(self._count_list)):
            if i==0:
                names[i]='%s in (-Inf, %d)'%(variable_name, self._thr_list[0])
            elif i==len(self._count_list)-1:
                names[i]='%s in [%d, +Inf)'%(variable_name, self._thr_list[-1])
            else:
                names[i]='%s in [%d, %d)'%(variable_name, self._thr_list[i-1], self._thr_list[i])
        return names
        
    def feed_data_list(self,data_list,is_select=False):
        if self._formula=='scalar':
            value_list=np.ravel(data_list) # flatten data_list into a line
        elif self._formula=='mean':
            value_list=np.mean(data_list, axis=1)
        
        stat_index=list(range(len(self._count_list)))
        for i in range(len(self._count_list)):
            if i==0:
                logic = (value_list < self._thr_list[0]).astype(int)
            elif i==len(self._count_list)-1:
                logic = (value_list >= self._thr_list[-1]).astype(int)
            else:
                logic_low = (value_list < self._thr_list[i]).astype(int)
                logic_high = (value_list >= self._thr_list[i-1]).astype(int)
                logic = np. multiply(logic_low,logic_high)
            self._count_list[i] = sum(logic) 
            if is_select==True:
                stat_index[i] = [idx for idx, e in enumerate(logic) if e==1]
        
        if(sum(self._count_list) != len(value_list)):
            print('WARNING:')
            print(sum(self._count_list))
            print(len(value_list))
            print(value_list)
        
        return self._count_list.astype(int), stat_index

class DataSet(object):
    def __init__(self, vectors, labels, efs, fake_data = False):
        # efs : external features, such as QP

        assert vectors.shape[0] == labels.shape[0], ('vectors.shape: %s labels.shape: %s' % (vectors.shape, labels.shape))
        self._num_examples = vectors.shape[0]

        vectors = vectors.astype(np.float32)
        labels = labels.astype(np.float32)
        efs = efs.astype(np.float32)
    
        self._vectors = vectors
        self._labels = labels
        self._efs = efs 
        self._epochs_completed = 0
        self._index_in_epoch = 0

    @property
    def vectors(self):
        return self._vectors

    @property
    def labels(self):
        return self._labels
  
    @property
    def efs(self):
        return self._efs  

    @property
    def num_examples(self):
        return self._num_examples

    @property
    def epochs_completed(self):
        return self._epochs_completed

    def next_batch(self, batch_size, fake_data=False):
        """Return the next `batch_size` examples from this data set."""
        start = self._index_in_epoch
        self._index_in_epoch += batch_size
        if self._index_in_epoch > self._num_examples:
            start = 0
            self._index_in_epoch = batch_size
            assert batch_size <= self._num_examples
        end = self._index_in_epoch
        return self._vectors[start:end], self._labels[start:end], self._efs[start:end] 
  
    def next_batch_random(self, batch_size, fake_data=False):
        batch_size_valid=self._num_examples
        if batch_size <= self._num_examples:
            batch_size_valid=batch_size
        index_list=np.random.randint(0,self._num_examples,[batch_size])
        return self._vectors[index_list], self._labels[index_list], self._efs[index_list]

def read_data_sets(train_skip_samples = 0, valid_skip_samples = 0, test_skip_samples = 0):
    global TRAIN_FILE_READER, VALID_FILE_READER, TEST_FILE_READER
    class DataSets(object):
        def __init__(self):
            self.train = []
            self.validation = []
            self.test = []    
    
    data_sets = DataSets() 
    get_train_valid_test_sets(DATA_SWITCH)
    
    TRAIN_FILE_READER = fr.FileReader()
    TRAIN_FILE_READER.initialize(os.path.join(DATA_PATH,TRAINSET), TRAINSET_MAXSIZE * NUM_SAMPLE_LENGTH)
    TRAIN_FILE_READER.read_data_skip(train_skip_samples * NUM_SAMPLE_LENGTH, False, np.uint8)
    
    VALID_FILE_READER = fr.FileReader()
    VALID_FILE_READER.initialize(os.path.join(DATA_PATH,VALIDSET), VALIDSET_MAXSIZE * NUM_SAMPLE_LENGTH)
    VALID_FILE_READER.read_data_skip(valid_skip_samples * NUM_SAMPLE_LENGTH, False, np.uint8)
    
    TEST_FILE_READER = fr.FileReader()
    TEST_FILE_READER.initialize(os.path.join(DATA_PATH,TESTSET), TESTSET_MAXSIZE * NUM_SAMPLE_LENGTH)
    TEST_FILE_READER.read_data_skip(test_skip_samples * NUM_SAMPLE_LENGTH, False, np.uint8)

    change_train_data_set(data_sets)
    change_valid_data_set(data_sets)
    change_test_data_set(data_sets)
    
    return data_sets

def change_train_data_set(data_sets):
    data_sets.train = get_data_set(TRAIN_FILE_READER, TRAINSET_READSIZE)

def change_valid_data_set(data_sets):
    data_sets.validation = get_data_set(VALID_FILE_READER, VALIDSET_READSIZE)
    
def change_test_data_set(data_sets):
    data_sets.test = get_data_set(TEST_FILE_READER, TESTSET_READSIZE)
