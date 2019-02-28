import net_CTU64 as nt
import file_reader as fr

import random
import gzip
import os
import numpy as np
import tensorflow as tf

data_dir = 'Data/' # path of training/validation/test data

IMAGE_SIZE = nt.IMAGE_SIZE 
NUM_CHANNELS = nt.NUM_CHANNELS 
NUM_EXT_FEATURES = nt.NUM_EXT_FEATURES
NUM_LABEL_BYTES = nt.NUM_LABEL_BYTES
NUM_SAMPLE_LENGTH = IMAGE_SIZE * IMAGE_SIZE * NUM_CHANNELS + 64 + (51 + 1) * NUM_LABEL_BYTES

DEFAULT_THR_LIST = [0.5, 1.5, 2.5]

PARTLY_TUNING_MODE = 0

TRAINSET_MAXSIZE = 10000000
VALIDSET_MAXSIZE = 10000000
TESTSET_MAXSIZE  = 10000000

TRAINSET_READSIZE = 80000
VALIDSET_READSIZE = 60000
TESTSET_READSIZE  = 60000

TRAIN_FILE_READER = []
VALID_FILE_READER = []
TEST_FILE_READER  = []

TRAINSET = []
VALIDSET = []
TESTSET  = []

# select training for which range of QP
MODEL_TYPE = 1

if MODEL_TYPE == 1:
    MODEL_NAME = 'qp22'
    SELECT_QP_LIST = [22]
    EVALUATE_QP_THR_LIST = [20, 25]
if MODEL_TYPE == 2:
    MODEL_NAME = 'qp27'
    SELECT_QP_LIST = [27]
    EVALUATE_QP_THR_LIST = [25, 30]
if MODEL_TYPE == 3:
    MODEL_NAME = 'qp32'
    SELECT_QP_LIST = [32]
    EVALUATE_QP_THR_LIST = [30, 35]
if MODEL_TYPE == 4:
    MODEL_NAME = 'qp37'
    SELECT_QP_LIST = [37]
    EVALUATE_QP_THR_LIST = [35, 40]

DATA_SWITCH = 1

def get_train_valid_test_sets(DATA_SWITCH):
    global TRAINSET, VALIDSET, TESTSET
    
    if DATA_SWITCH == 0: # full sets (the full program for generating these files will be open on GitHub soon)
        TRAINSET = 'AI_Train_2446725.dat_shuffled'
        VALIDSET = 'AI_Valid_143925.dat_shuffled'
        TESTSET  = 'AI_Test_287850.dat_shuffled'
    elif DATA_SWITCH == 1: # demo sets
        TRAINSET = 'AI_Train_5000.dat_shuffled'
        VALIDSET = 'AI_Valid_5000.dat_shuffled'
        TESTSET  = 'AI_Test_5000.dat_shuffled'
    elif DATA_SWITCH == 2: # choose other files if necessary
        pass

def getFileSize(path):
    try:
        size = os.path.getsize(path)
        return size
    except Exception as err:
        print(err)

def _read32(bytestream):
    dt = np.dtype(np.uint32).newbyteorder('>')
    return np.frombuffer(bytestream.read(4), dtype=dt)[0]
  
def read32(f):
    byte0=ord(f.read(1)[0])
    byte1=ord(f.read(1)[0])
    byte2=ord(f.read(1)[0])
    byte3=ord(f.read(1)[0])  
    return byte0+(byte1<<8)+(byte2<<16)+(byte3<<24)   

def get_data_set(file_reader, read_bytes, is_loop=True, dtype=np.uint8 , is_show_stat=False):
    
    data = file_reader.read_data(read_bytes, isloop=is_loop, dtype=dtype)
    data_bytes=len(data)
    assert data_bytes % NUM_SAMPLE_LENGTH == 0
    num_samples = int(data_bytes / NUM_SAMPLE_LENGTH)
    
    data = data.reshape(num_samples, NUM_SAMPLE_LENGTH)
    
    images = data[:,0:4096].astype(np.float32)
    images = np.reshape(images, [-1, IMAGE_SIZE, IMAGE_SIZE, NUM_CHANNELS])
    
    qps = np.random.choice(SELECT_QP_LIST,size=num_samples)
    qps = qps.reshape(num_samples, NUM_EXT_FEATURES)
    
    labels = np.zeros((num_samples, NUM_LABEL_BYTES))
    for i in range(num_samples):
        labels[i,:]=data[i, 4160+qps[i,0]*NUM_LABEL_BYTES:4160+(qps[i,0]+1)*NUM_LABEL_BYTES]

    if is_show_stat==True:
        range_stat = RangingStatistics(DEFAULT_THR_LIST, 'scalar')
        count_list_ori, _ = range_stat.feed_data_list(labels)
        print(count_list_ori)
        
    return DataSet(images, labels, qps)

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
                names[i] = '%s in (-Inf, %d)'%(variable_name, self._thr_list[0])
            elif i==len(self._count_list)-1:
                names[i] = '%s in [%d, +Inf)'%(variable_name, self._thr_list[-1])
            else:
                names[i] = '%s in [%d, %d)'%(variable_name, self._thr_list[i-1], self._thr_list[i])
        return names
        
    def feed_data_list(self,data_list,is_select=False):
        if self._formula=='scalar':
            value_list=np.ravel(data_list)
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
        
        assert sum(self._count_list)==len(value_list)
        
        return self._count_list.astype(int), stat_index

class DataSet(object):
    def __init__(self, images, labels, qps, fake_data=False, dtype=tf.float32):
        dtype = tf.as_dtype(dtype).base_dtype
        if dtype not in (tf.uint8, tf.float32):
            raise TypeError('Invalid image dtype %r, expected uint8 or float32' %
                      dtype)
        if fake_data:
            self._num_examples = 10000
        else:
            assert images.shape[0] == labels.shape[0], ('images.shape: %s labels.shape: %s' % (images.shape, labels.shape))
            self._num_examples = images.shape[0]
            images = images.astype(np.float32)
            labels = labels.astype(np.int32)
            qps = qps.astype(np.float32)
    
        self._images = images
        self._labels = labels
        self._qps = qps
        self._epochs_completed = 0
        self._index_in_epoch = 0

    @property
    def images(self):
        return self._images

    @property
    def labels(self):
        return self._labels
  
    @property
    def qps(self):
        return self._qps  

    @property
    def num_examples(self):
        return self._num_examples

    @property
    def epochs_completed(self):
        return self._epochs_completed

    def next_batch(self, batch_size, fake_data=False):
        start = self._index_in_epoch
        self._index_in_epoch += batch_size
        if self._index_in_epoch > self._num_examples:
            start = 0
            self._index_in_epoch = batch_size
            assert batch_size <= self._num_examples
        end = self._index_in_epoch
        return self._images[start:end], self._labels[start:end], self._qps[start:end] 
  
    def next_batch_random(self, batch_size, fake_data=False):
        
        batch_size_valid=self._num_examples
        if batch_size <= self._num_examples:
            batch_size_valid=batch_size
        index_list = np.random.randint(0, self._num_examples,[batch_size_valid])
        return self._images[index_list], self._labels[index_list], self._qps[index_list]

def read_data_sets(fake_data=False):
    global TRAIN_FILE_READER, VALID_FILE_READER, TEST_FILE_READER

    class DataSets(object):
        pass

    data_sets = DataSets()
    data_sets.train = []
    data_sets.validation = []
    data_sets.test = []
    data_sets.trainpart = []
    data_sets.testpart = []
    
    get_train_valid_test_sets(DATA_SWITCH)

    TRAIN_FILE_READER = fr.FileReader()
    TRAIN_FILE_READER.initialize(os.path.join(data_dir, TRAINSET), TRAINSET_MAXSIZE * NUM_SAMPLE_LENGTH)

    VALID_FILE_READER = fr.FileReader()
    VALID_FILE_READER.initialize(os.path.join(data_dir, VALIDSET), VALIDSET_MAXSIZE * NUM_SAMPLE_LENGTH)

    TEST_FILE_READER = fr.FileReader()
    TEST_FILE_READER.initialize(os.path.join(data_dir, TESTSET), TESTSET_MAXSIZE * NUM_SAMPLE_LENGTH)

    change_train_data_set(data_sets)
    change_valid_data_set(data_sets)
    change_test_data_set(data_sets)
    
    return data_sets

def change_train_data_set(data_sets):
    global TRAIN_FILE_READER
    data_sets.train = get_data_set(TRAIN_FILE_READER, TRAINSET_READSIZE * NUM_SAMPLE_LENGTH, is_show_stat=True)

def change_valid_data_set(data_sets):
    global VALID_FILE_READER
    data_sets.validation = get_data_set(VALID_FILE_READER, VALIDSET_READSIZE * NUM_SAMPLE_LENGTH, is_show_stat=True)
    
def change_test_data_set(data_sets):
    global TEST_FILE_READER
    data_sets.test = get_data_set(TEST_FILE_READER, TESTSET_READSIZE * NUM_SAMPLE_LENGTH, is_show_stat=True)
