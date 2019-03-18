import net_CTU64 as nt
import file_reader as fr

import random
import gzip
import os
import tensorflow.python.platform
import numpy
from six.moves import urllib
from six.moves import xrange
import tensorflow as tf

import matplotlib.pyplot as plt 
from matplotlib.pyplot import plot,savefig 
import matplotlib.patches as patches

DATA_PATH = '/media/F/Data/'

IMAGE_SIZE = nt.IMAGE_SIZE
NUM_CHANNELS = nt.NUM_CHANNELS
NUM_EXT_FEATURES = nt.NUM_EXT_FEATURES
NUM_LABEL_BYTES = nt.NUM_LABEL_BYTES
NUM_SAMPLE_LENGTH = 0

IS_TRAIN_SELECT = False
IS_VALID_SELECT = False
IS_TEST_SELECT = False

DEFAULT_THR_LIST = [0.5, 1.5, 2.5]

PARTLY_TUNING_MODE = 0

TRAINSET_MAXSIZE = 10000000
VALIDSET_MAXSIZE = 10000000
TESTSET_MAXSIZE = 10000000

TRAINSET_READSIZE = 50000
VALIDSET_READSIZE = 25000
TESTSET_READSIZE = 25000

train_file_reader = []
valid_file_reader = []
test_file_reader = []

TRAINSET = []
VALIDSET = []
TESTSET = []

NUM_QPS = 4
NUM_BYTES_PER_QP = 1 + NUM_LABEL_BYTES + IMAGE_SIZE * IMAGE_SIZE * NUM_CHANNELS
NUM_SAMPLE_LENGTH = NUM_BYTES_PER_QP * NUM_QPS + 64

MODEL_TYPE = 0
if MODEL_TYPE == 0:
    MODEL_NAME = 'qp22~37'
    SELECT_QP_LIST = [22,27,32,37]
    EVALUATE_QP_THR_LIST = [25, 30, 35]
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
    
DATA_SWITCH = 0
# above are global variables ------------------------------------------
def get_train_valid_test_sets(DATA_SWITCH):
    global TRAINSET, VALIDSET, TESTSET

    if DATA_SWITCH == 0: # original full set
        TRAINSET = 'LDP_Train_9011161.dat_shuffled'
        VALIDSET = 'LDP_Valid_1057660.dat_shuffled'
        TESTSET  = 'LDP_Test_1896004.dat_shuffled'
    if DATA_SWITCH == 1: # demo set
        TRAINSET = 'LDP_Train_20000.dat_shuffled'
        VALIDSET = 'LDP_Train_10000.dat_shuffled'
        TESTSET  = 'LDP_Train_10000.dat_shuffled'
    if DATA_SWITCH == 2: # switch to other files
        TRAINSET = '...'
        VALIDSET = '...'
        TESTSET  = '...'

def translabel_matrix(label_matrix):
    return label_matrix

def getFileSize(path):
    try:
        size = os.path.getsize(path)
        return size
    except Exception as err:
        print(err)

def _read32(bytestream):
    dt = numpy.dtype(numpy.uint32).newbyteorder('>')
    return numpy.frombuffer(bytestream.read(4), dtype=dt)[0]
  
def read32(f):
    byte0 = ord(f.read(1)[0])
    byte1 = ord(f.read(1)[0])
    byte2 = ord(f.read(1)[0])
    byte3 = ord(f.read(1)[0])
    return byte0 + (byte1 << 8 ) + (byte2 << 16) + (byte3 << 24)

def get_data_set(file_reader, read_bytes, is_loop = True, dtype = numpy.uint8 , is_show_stat = False, is_select = False):
    
    data = file_reader.read_data(read_bytes, isloop = is_loop, dtype = dtype)
    data_bytes = len(data)
    assert data_bytes % NUM_SAMPLE_LENGTH == 0
    num_samples = int(data_bytes / NUM_SAMPLE_LENGTH)
    
    data = data.reshape(num_samples, NUM_SAMPLE_LENGTH)
     
    info_block = numpy.zeros((num_samples, NUM_BYTES_PER_QP))
    qps_index = numpy.random.choice(NUM_QPS,size=num_samples)
    
    for i in range(num_samples):       
        info_block[i,:] = data[i, 64 + NUM_BYTES_PER_QP * qps_index[i] : 64 + NUM_BYTES_PER_QP * (1 + qps_index[i])]
    
    images_select = info_block[:, NUM_EXT_FEATURES + NUM_LABEL_BYTES : ]
    images_select = numpy.reshape(images_select, [-1, IMAGE_SIZE, IMAGE_SIZE, 1])
    labels_select = info_block[:, NUM_EXT_FEATURES : NUM_EXT_FEATURES + NUM_LABEL_BYTES]
    qps_select = info_block[:, : NUM_EXT_FEATURES]
    others_select = data[:, 0:64];
        
    if is_show_stat==True:
        range_stat = RangingStatistics(DEFAULT_THR_LIST, 'scalar')
        count_list_select, _ = range_stat.feed_data_list(labels_select)
        print(count_list_select)
        
    return DataSet(images_select, labels_select, qps_select, others_select)

class RangingStatistics(object):
    def __init__(self, thr_list, formula):
        self._thr_list=thr_list
        self._formula=formula
        self._count_list=numpy.zeros((len(thr_list)+1))
    
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
        self._count_list=numpy.zeros((len(self._thr_list)+1))
        
    def set_thr_list(self,thr_list):
        self._thr_list=thr_list
        self._count_list=numpy.zeros((len(thr_list)+1))
        
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
            value_list=numpy.ravel(data_list)
        elif self._formula=='mean':
            value_list=numpy.mean(data_list, axis=1)
        
        stat_index=list(range(len(self._count_list)))
        for i in range(len(self._count_list)):
            if i==0:
                logic = (value_list < self._thr_list[0]).astype(int)
            elif i==len(self._count_list)-1:
                logic = (value_list >= self._thr_list[-1]).astype(int)
            else:
                logic_low = (value_list < self._thr_list[i]).astype(int)
                logic_high = (value_list >= self._thr_list[i-1]).astype(int)
                logic = numpy. multiply(logic_low,logic_high)
            self._count_list[i] = sum(logic) 
            if is_select==True:
                stat_index[i] = [idx for idx, e in enumerate(logic) if e==1]
        
        assert sum(self._count_list)==len(value_list)
        
        return self._count_list.astype(int), stat_index

class DataSet(object):
    def __init__(self, images, labels, qps, others, fake_data=False, dtype=tf.float32):
        dtype = tf.as_dtype(dtype).base_dtype
        if dtype not in (tf.uint8, tf.float32):
            raise TypeError('Invalid image dtype %r, expected uint8 or float32' %
                      dtype)
        if fake_data:
            self._num_examples = 10000
        else:
            assert images.shape[0] == labels.shape[0], ('images.shape: %s labels.shape: %s' % (images.shape, labels.shape))
            self._num_examples = images.shape[0]

            images = images.astype(numpy.float32)
            labels = labels.astype(numpy.int32)
            qps = qps.astype(numpy.float32)
            others = others.astype(numpy.int32)
    
        self._images = images
        self._labels = labels
        self._qps = qps
        self._others = others
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
    def others(self):
        return self._others    

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
        return self._images[start:end], self._labels[start:end], self._qps[start:end], self._others[start:end]
  
    def next_batch_random(self, batch_size, fake_data = False):
        batch_size_valid = self._num_examples
        if batch_size <= self._num_examples:
            batch_size_valid = batch_size
        index_list = numpy.random.randint(0,self._num_examples,[batch_size_valid])
        return self._images[index_list], self._labels[index_list], self._qps[index_list], self._others[index_list]

def read_data_sets(fake_data=False, dtype=tf.float32):
    global train_file_reader, valid_file_reader, test_file_reader
    class DataSets(object):
        pass
    data_sets = DataSets()

    if fake_data:
        def fake():
            return DataSet([], [], fake_data=True, dtype=dtype)
        data_sets.train = fake()
        data_sets.validation = fake()
        data_sets.test = fake()
        return data_sets 
    
    get_train_valid_test_sets(DATA_SWITCH)

    train_file_reader = fr.FileReader()
    valid_file_reader = fr.FileReader()
    test_file_reader = fr.FileReader()

    train_file_reader.initialize(os.path.join(DATA_PATH,TRAINSET), TRAINSET_MAXSIZE * NUM_SAMPLE_LENGTH)
    valid_file_reader.initialize(os.path.join(DATA_PATH,VALIDSET), VALIDSET_MAXSIZE * NUM_SAMPLE_LENGTH)
    test_file_reader.initialize(os.path.join(DATA_PATH,TESTSET), TESTSET_MAXSIZE * NUM_SAMPLE_LENGTH)

    change_train_data_set(data_sets)
    change_valid_data_set(data_sets)
    change_test_data_set(data_sets)
    
    return data_sets

def change_train_data_set(data_sets):
    data_sets.train = get_data_set(train_file_reader, TRAINSET_READSIZE * NUM_SAMPLE_LENGTH, is_show_stat = True, is_select = IS_TRAIN_SELECT)

def change_valid_data_set(data_sets):
    data_sets.validation = get_data_set(valid_file_reader, VALIDSET_READSIZE * NUM_SAMPLE_LENGTH, is_show_stat = True, is_select = IS_VALID_SELECT)
    
def change_test_data_set(data_sets):
    data_sets.test = get_data_set(test_file_reader, TESTSET_READSIZE * NUM_SAMPLE_LENGTH, is_show_stat = True, is_select = IS_TEST_SELECT)

