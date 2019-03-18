import os,sys,shutil
import matplotlib
import matplotlib.pyplot as plt 
from matplotlib.pyplot import plot,savefig
import time
import numpy as np
import math
import input_data
import tensorflow as tf
import config as cf

matplotlib.rcParams['xtick.direction'] = 'out'  
matplotlib.rcParams['ytick.direction'] = 'out'

# decide to run with which type of device 
# 0:CPU  1:GPU (limited space)  2:GPU (unlimited space)
DEVICE_MODE = 1
if DEVICE_MODE == 0:
	os.environ['CUDA_VISIBLE_DEVICES'] = '' # GPU is disabled
	sess = tf.Session()
elif DEVICE_MODE == 1:
	config = tf.ConfigProto()
	config.gpu_options.per_process_gpu_memory_fraction = 0.25
	sess = tf.Session(config = config)
elif DEVICE_MODE == 2:
	sess = tf.Session()

NUM_CHANNELS = cf.NUM_CHANNELS
NUM_EXT_FEATURES = cf.NUM_EXT_FEATURES
NUM_LABEL_BYTES = cf.NUM_LABEL_BYTES
VECTOR_LENGTH = cf.VECTOR_LENGTH
LSTM_MAX_LENGTH = cf.LSTM_MAX_LENGTH
LSTM_OUTPUT_LENGTH = cf.LSTM_OUTPUT_LENGTH

BATCH_SIZE = cf.BATCH_SIZE

DATA_SWITCH = input_data.DATA_SWITCH

RUN_MODE = 0 # 0: train  1: evaluate
IS_RELOAD = False # whether to reload the model trained last time. True: train from scretch, False: fine-tune
IS_READ_DATA_SKIP = True # when IS_RELOAD == True, whether to skip a certain number of samples before reading training, validation and test data into RAM. 
IS_TRAIN_LIMIT_GRAD = True # whether to limit the gradient when training
IS_TRAIN_BALANCE = True # whether to balance the loss between that of positive and negative samples. 
LEARNING_RATE_INIT = 0.1 # initial learning rate
IS_RESET_LEARNING_RATE = False # when IS_RELOAD == True, whether to set learning rate as LEARNING_RATE_INIT 
MOMENTUM_INIT = 0.9 # initial momentum

ITER_TIMES = 200000 # number of iterations in this execution

# change learning rate exponentially to 0.3163 of the perious rate, every 25000 iterations
ITER_TIMES_PER_CHANGE_RATE = 25000
LEARNING_RATE_DECAY_RATIO = 0.3163 

EVALUATE_QP_THR_LIST = input_data.EVALUATE_QP_THR_LIST
MODEL_NAME = input_data.MODEL_NAME

NUM_TRAIN_PART = 10000
NUM_VALID_PART = 10000

ITER_TIMES_PER_PRINT = 1000
ITER_TIMES_PER_EVALUATE = 1000
ITER_TIMES_PER_CHANGE_TRAINSET = 1000
ITER_TIMES_PER_CHANGE_VALIDSET = 5000
ITER_TIMES_PER_SAVE = 10000

ITER_TIMES_PER_EVALUATE_LARGE_BATCH = 25000
	
def is_sep_64(y_one_sample, thr):
	depth_mean = np.mean(y_one_sample)
	if depth_mean > thr:
		return 1
	else:
		return 0
	
def is_sep_32(y_one_sample, thr):
	depth_mean = np.mean(y_one_sample)
	if depth_mean > thr:
		return 1
	else:
		return 0

def is_sep_16(y_one_sample, thr):
	if y_one_sample > thr:
		return 1
	else:
		return 0
	
def get_class_matrices(y_truth, y_predict_64, y_predict_32, y_predict_16, thr_list):
	
	# y_truth is N*16 matrix, with elements 0,1,2 or 3, indicating CU depths, where N is the number of samples.
	# y_predict_16 is N*16 matrix, with elements 0 or 1, indicating whether 16*16 CUs are seperated into 8*8 CUs.
	# return 3 2*2 matrices, representing the classification matrices of:
	#     whether to split a 64*64 CU into 32*32 CUs (non-split: class 0, split: class 1)
	#     whether to split a 32*32 CU into 16*16 CUs (non-split: class 0, split: class 1)
	#     whether to split a 16*16 CU into  8* 8 CUs (non-split: class 0, split: class 1)
	# in every classification matrix [[n00 n01] 
	#                                 [n10 n11]],
	# nxy indicates the number of samples with ground-truth of x and predicting label of y
	
	matrix_64 = [[0, 0], [0, 0]]
	matrix_32 = [[0, 0], [0, 0]]
	matrix_16 = [[0, 0], [0, 0]]
	assert y_truth.shape[0] == y_predict_16.shape[0]
	num_samples=y_truth.shape[0]
	index_32_list=[[0,1,4,5],[2,3,6,7],[8,9,12,13],[10,11,14,15]]
	
	for i in range(num_samples):
		class_64_truth=is_sep_64(y_truth[i], input_data.DEFAULT_THR_LIST[0])
		class_64_predict=is_sep_64(y_predict_64[i], thr_list[0])
		matrix_64[class_64_truth][class_64_predict] += 1
		if class_64_truth==1:
			for j in range(4):
				class_32_truth=is_sep_32(y_truth[i][index_32_list[j]], input_data.DEFAULT_THR_LIST[1])
				class_32_predict=is_sep_32(y_predict_32[i][j], thr_list[1])
				matrix_32[class_32_truth][class_32_predict] += 1
				if class_32_truth==1:
					for k in range(4):
						class_16_truth=is_sep_16(y_truth[i][index_32_list[j][k]], input_data.DEFAULT_THR_LIST[2])
						class_16_predict=is_sep_16(y_predict_16[i][index_32_list[j][k]], thr_list[2])
						matrix_16[class_16_truth][class_16_predict] += 1
						
	return matrix_64, matrix_32, matrix_16

def get_tendency_2x2(matrix_2x2):
	if matrix_2x2[0][1]==0 and matrix_2x2[1][0]==0:
		return 0
	elif matrix_2x2[0][1]==0 or matrix_2x2[1][1]==0:
		return -100
	elif matrix_2x2[1][0]==0 or matrix_2x2[0][0]==0:
		return 100
	else:
		return -math.log10((matrix_2x2[0][0] / matrix_2x2[0][1]) / (matrix_2x2[1][1] / matrix_2x2[1][0]))

def get_accuracy_on_large_data_interqp(f, data_set, qp_thr_list, thr_list, plot_tag_set=None):
	range_stat=input_data.RangingStatistics(qp_thr_list, 'scalar')
	qps = data_set.efs[:,1]
	count_list, stat_index=range_stat.feed_data_list(qps, is_select=True)
	segment_names = range_stat.get_segment_names('QP')
	for i in range(len(qp_thr_list)+1):
		if count_list[i]>0:
			fprint(f, '------------------------------')
			fprint(f, segment_names[i])
			accuracy,  _, _, _, accuracy_list, tendency_list = get_accuracy_on_large_data(f, data_set.vectors[stat_index[i]], data_set.efs[stat_index[i]], data_set.labels[stat_index[i]], thr_list=thr_list, is_get_y_predict=True, is_print=True, plot_tag='%s_%d'%(plot_tag_set,i))

def get_accuracy_on_large_data(f,vectors,efs,labels,thr_list,is_get_y_predict=False,is_print=False,plot_tag=None):
	length=np.shape(vectors)[0]
	PRE_BATCH_SIZE=5000
	
	accuracy_total=np.zeros((3))
	num_total=0
	y_predict_16=[]
	y_predict_32=[]
	y_predict_64=[]
	matrix_64_sum = [[0, 0], [0, 0]]
	matrix_32_sum = [[0, 0], [0, 0]]
	matrix_16_sum = [[0, 0], [0, 0]]
	
	for i in range(math.ceil(length/PRE_BATCH_SIZE)):
		index_start=PRE_BATCH_SIZE*i
		index_end=PRE_BATCH_SIZE*(i+1)
		if index_end > length:
			index_end = length
		y_truth_valid_temp,y_predict_temp_64,y_predict_temp_32,y_predict_temp_16, accuracy_temp = sess.run([y_truth_valid,y_pred_flat_64,y_pred_flat_32,y_pred_flat_16,accuracy_list],feed_dict={x:vectors[index_start:index_end], y_:labels[index_start:index_end], ef:efs[index_start:index_end], isdrop:0})
		if is_get_y_predict==True:
			if y_predict_16==[]:
				y_predict_16=y_predict_temp_16
			else:
				y_predict_16 = np.vstack((y_predict_16, y_predict_temp_16))
			if y_predict_32==[]:
				y_predict_32=y_predict_temp_32
			else:
				y_predict_32 = np.vstack((y_predict_32, y_predict_temp_32))
			if y_predict_64==[]:
				y_predict_64=y_predict_temp_64
			else:
				y_predict_64 = np.vstack((y_predict_64, y_predict_temp_64))		
		accuracy_total += accuracy_temp * (index_end-index_start)
		matrix_64_temp, matrix_32_temp, matrix_16_temp = get_class_matrices(y_truth_valid_temp,y_predict_temp_64,y_predict_temp_32,y_predict_temp_16,thr_list)
		matrix_64_sum = np.add(matrix_64_sum, matrix_64_temp)
		matrix_32_sum = np.add(matrix_32_sum, matrix_32_temp)
		matrix_16_sum = np.add(matrix_16_sum, matrix_16_temp)		
	
	accuracy_64 = (matrix_64_sum[0][0] + matrix_64_sum[1][1])/np.sum(matrix_64_sum)
	accuracy_32 = (matrix_32_sum[0][0] + matrix_32_sum[1][1])/np.sum(matrix_32_sum)
	accuracy_16 = (matrix_16_sum[0][0] + matrix_16_sum[1][1])/np.sum(matrix_16_sum)

	tendency_64 = get_tendency_2x2(matrix_64_sum)
	tendency_32 = get_tendency_2x2(matrix_32_sum)
	tendency_16 = get_tendency_2x2(matrix_16_sum)
	if is_print==True:
		print(matrix_64_sum)
		print(matrix_32_sum)
		print(matrix_16_sum)
		if f!=None:
			fprint(f, 'accuracy = %lf, %lf, %lf'%(accuracy_64,accuracy_32,accuracy_16))
			fprint(f, 'tendency = %lf, %lf, %lf'%(tendency_64,tendency_32,tendency_16))
	accuracy_all=accuracy_total/length
	accuracy_valid=[accuracy_64, accuracy_32, accuracy_16]
	tendency_valid=[tendency_64, tendency_32, tendency_16]
	
	return accuracy_all, y_predict_64, y_predict_32, y_predict_16, accuracy_valid, tendency_valid

def evaluate_loss_accuracy(step,learning_rate_value):
	global train_accuracy_list,valid_accuracy_list,train_loss_list,valid_loss_list
	train_batch = data_sets.train.next_batch_random(NUM_TRAIN_PART)
	valid_batch = data_sets.validation.next_batch_random(NUM_VALID_PART)
	
	train_y_truth_temp,train_y_predict_temp_64,train_y_predict_temp_32,train_y_predict_temp_16,train_accuracy_list_c, train_loss_list_c = sess.run([y_truth_valid,y_pred_flat_64,y_pred_flat_32,y_pred_flat_16, accuracy_list, loss_list],feed_dict={x:train_batch[0], y_:train_batch[1], ef:train_batch[2], isdrop:0})			
	valid_y_truth_temp,valid_y_predict_temp_64,valid_y_predict_temp_32,valid_y_predict_temp_16,valid_accuracy_list_c, valid_loss_list_c = sess.run([y_truth_valid,y_pred_flat_64,y_pred_flat_32,y_pred_flat_16, accuracy_list, loss_list],feed_dict={x:valid_batch[0], y_:valid_batch[1], ef:valid_batch[2], isdrop:0})
	
	train_accuracy_list.append(list(train_accuracy_list_c))
	valid_accuracy_list.append(list(valid_accuracy_list_c))
	train_loss_list.append(list(train_loss_list_c))
	valid_loss_list.append(list(valid_loss_list_c))
	step_list.append(step)
	
	matrix_64, matrix_32, matrix_16 = get_class_matrices(train_y_truth_temp,train_y_predict_temp_64,train_y_predict_temp_32,train_y_predict_temp_16,[0.5, 0.5, 0.5])
	train_tendency_64 = get_tendency_2x2(matrix_64)
	train_tendency_32 = get_tendency_2x2(matrix_32)
	train_tendency_16 = get_tendency_2x2(matrix_16)
	matrix_64, matrix_32, matrix_16 = get_class_matrices(valid_y_truth_temp,valid_y_predict_temp_64,valid_y_predict_temp_32,valid_y_predict_temp_16,[0.5, 0.5, 0.5])
	valid_tendency_64 = get_tendency_2x2(matrix_64)
	valid_tendency_32 = get_tendency_2x2(matrix_32)
	valid_tendency_16 = get_tendency_2x2(matrix_16)	
	str_loss_accu = ('%s step %d: loss=[[%.3f %.3f %.3f] [%.3f %.3f %.3f]], accu=[[%.3f %.3f %.3f] [%.3f %.3f %.3f]], lr=%g' %(get_time_str(), step, train_loss_list_c[0],train_loss_list_c[1],train_loss_list_c[2],valid_loss_list_c[0],valid_loss_list_c[1],valid_loss_list_c[2], train_accuracy_list_c[0],train_accuracy_list_c[1],train_accuracy_list_c[2], valid_accuracy_list_c[0],valid_accuracy_list_c[1],valid_accuracy_list_c[2], learning_rate_value))		
	str_tendency = ('tendency = [[%.3f, %.3f, %.3f] [%.3f, %.3f, %.3f]]'%(train_tendency_64,train_tendency_32,train_tendency_16,valid_tendency_64,valid_tendency_32,valid_tendency_16))	
	print('%s, %s'%(str_loss_accu, str_tendency))
	train_tendency_list_c = [train_tendency_64, train_tendency_32, train_tendency_16]
	valid_tendency_list_c = [valid_tendency_64, valid_tendency_32, valid_tendency_16]
	train_tendency_list.append(list(train_tendency_list_c))
	valid_tendency_list.append(list(valid_tendency_list_c))
	
	# valid_accuracy_16_list = np.zeros((LSTM_OUTPUT_LENGTH))
	# # print accuracy for each time step:
	# for t in range(LSTM_OUTPUT_LENGTH):
	# 	valid_y_truth_temp_current = valid_y_truth_temp[t:-1:LSTM_OUTPUT_LENGTH]
	# 	matrix_64, matrix_32, matrix_16 = get_class_matrices(valid_y_truth_temp[t:-1:LSTM_OUTPUT_LENGTH],valid_y_predict_temp_64[t:-1:LSTM_OUTPUT_LENGTH],valid_y_predict_temp_32[t:-1:LSTM_OUTPUT_LENGTH],valid_y_predict_temp_16[t:-1:LSTM_OUTPUT_LENGTH],[0.5, 0.5, 0.5])
	# 	valid_accuracy_16 = (matrix_16[0][0] + matrix_16[1][1])/np.sum(matrix_16)
	# 	valid_tendency_16 = get_tendency_2x2(matrix_16)
	# 	valid_accuracy_16_list[t] = valid_accuracy_16
	# 	print('t = %d, accuracy = %.3f, tendency %.3f' % (t, valid_accuracy_16, valid_tendency_16))
	# plot_accuracy_and_timestep("Evaluate/AccuTimeStep_%s_%d.png"%(get_time_str(),step), valid_accuracy_16_list)
	

def reload_loss_and_accuracy():
	global train_accuracy_list,valid_accuracy_list,train_loss_list,valid_loss_list
	file = open('Models/loss_accuracy_list.dat','r+')
	iter_times_last=int(file.readline())
	while True:
		line = file.readline()
		if not line:
			break
		line = line.replace('\r','').replace('\n','')
		str_arr=line.split('  ')
		if str_arr!=['']:
			step_list.append(int(str_arr[0]))
			train_loss_list.append(list([float(str_arr[1]),float(str_arr[2]),float(str_arr[3])]))
			valid_loss_list.append(list([float(str_arr[4]),float(str_arr[5]),float(str_arr[6])]))
			train_accuracy_list.append(list([float(str_arr[7]),float(str_arr[8]),float(str_arr[9])]))
			valid_accuracy_list.append(list([float(str_arr[10]),float(str_arr[11]),float(str_arr[12])]))
			train_tendency_list.append(list([float(str_arr[13]),float(str_arr[14]),float(str_arr[15])]))
			valid_tendency_list.append(list([float(str_arr[16]),float(str_arr[17]),float(str_arr[18])]))			
			pass 
		
	file.close()
	return iter_times_last

def get_time_str():
	return time.strftime('%Y%m%d_%H%M%S',time.localtime(time.time()))
	
def plot_accuracy_and_timestep(save_path, accuracy_list):
	fig=plt.figure(figsize=[10,6])
	ax=plt.subplot(111)
	
	ax.plot(np.asarray(range(len(accuracy_list))), accuracy_list) 
	ax.set_xlabel('Time Step')
	plt.title('Valid Accuracy')
	plt.ylim(0.5,0.8)
	
	savefig(save_path)
	plt.close(fig)	
	
def fprint(f, str):
	print(str)
	f.write(str+'\r\n')
	
def evaluate(is_optimize=False):	
	f = open('Models/log_%s.dat'%get_time_str(), 'w+')	
	thr_list = [0.5, 0.5, 0.5]
	get_accuracy_on_large_data_interqp(f, data_sets.train, EVALUATE_QP_THR_LIST, thr_list, plot_tag_set='train')
	get_accuracy_on_large_data_interqp(f, data_sets.validation, EVALUATE_QP_THR_LIST, thr_list, plot_tag_set='valid')
	get_accuracy_on_large_data_interqp(f, data_sets.test, EVALUATE_QP_THR_LIST, thr_list, plot_tag_set='test')
	f.close()
	
# placeholders to create the model

x = tf.placeholder("float", [None, LSTM_MAX_LENGTH, VECTOR_LENGTH])
y_ = tf.placeholder("float", [None, LSTM_MAX_LENGTH, NUM_LABEL_BYTES])
ef = tf.placeholder("float", [None, NUM_EXT_FEATURES])
isdrop = tf.placeholder("float")
global_step = tf.placeholder("float")

y_truth_valid, y_flat_64, y_flat_32, y_flat_16, y_pred_flat_64, y_pred_flat_32, y_pred_flat_16, total_loss, loss_list, learning_rate_current, train_step, accuracy_list, opt_vars_all, watcher_1, watcher_2, watcher_3 = input_data.nt.net(x,y_,ef,isdrop,global_step, LEARNING_RATE_INIT, MOMENTUM_INIT, ITER_TIMES_PER_CHANGE_RATE,LEARNING_RATE_DECAY_RATIO, limit_grad = IS_TRAIN_LIMIT_GRAD, is_balance = IS_TRAIN_BALANCE) 

saver = tf.train.Saver(opt_vars_all, write_version=tf.train.SaverDef.V2)
	
if RUN_MODE == 1: # evaluate
	data_sets = input_data.read_data_sets(0, 0, 0)
	saver.restore(sess, 'Models/model.dat')
	evaluate()

elif RUN_MODE == 0: # train
	
	train_accuracy_list=list()
	valid_accuracy_list=list()
	train_loss_list=list()
	valid_loss_list=list()	
	train_tendency_list=list()
	valid_tendency_list=list()
	step_list=list()	
	
	sess.run(tf.global_variables_initializer())
	if IS_RELOAD==True:
		saver.restore(sess, 'Models/model.dat')
		iter_times_last = reload_loss_and_accuracy()
	else: 	
		iter_times_last = 0
	
	print('iter_times_last = %d' % iter_times_last)
	
	if IS_READ_DATA_SKIP == True:
		train_skip_samples = iter_times_last // ITER_TIMES_PER_CHANGE_TRAINSET * input_data.TRAINSET_READSIZE
		valid_skip_samples = iter_times_last // ITER_TIMES_PER_CHANGE_VALIDSET * input_data.VALIDSET_READSIZE
		data_sets = input_data.read_data_sets(train_skip_samples, valid_skip_samples, 0)
	else:
		data_sets = input_data.read_data_sets(0, 0, 0)
	
	# the initial evaluation is only valid when starting a new training process (without fine tuning)
	# otherwise, the data have been saved in last training process, with no need to duplicate
	if IS_RELOAD==False:
		evaluate_loss_accuracy(iter_times_last,LEARNING_RATE_INIT)
	
	for i in range(ITER_TIMES):
		step = i+iter_times_last + 1
		
		batch = data_sets.train.next_batch_random(BATCH_SIZE)
		
		if IS_RESET_LEARNING_RATE == True:
			feed_step = i+1
		else:
			feed_step = step
			
		learning_rate_value, _=sess.run([learning_rate_current,train_step],feed_dict={x:batch[0], ef:batch[2], y_:batch[1], isdrop:1, global_step:feed_step})	

		if step % ITER_TIMES_PER_EVALUATE == 0:
			evaluate_loss_accuracy(step,learning_rate_value)	
		elif step % ITER_TIMES_PER_PRINT == 0:
			print ("%s  step %d" % (get_time_str(),step))
			
		if step % ITER_TIMES_PER_SAVE == 0:
			saver.save(sess, 'Models/model_%s_%d_%s.dat'%(get_time_str(),step,MODEL_NAME))			
			
		if step % ITER_TIMES_PER_CHANGE_TRAINSET == 0 and i!=ITER_TIMES-1:
			input_data.change_train_data_set(data_sets)
			
		if step % ITER_TIMES_PER_CHANGE_VALIDSET == 0 and i!=ITER_TIMES-1:
			input_data.change_valid_data_set(data_sets)
			
		if step % ITER_TIMES_PER_EVALUATE_LARGE_BATCH == 0 and i!=ITER_TIMES-1:
			evaluate()
		
	evaluate()
	
	# save the data plot as an image file
	fig=plt.figure(figsize=[20,8])
	
	ax=plt.subplot(131)
	step_list_every_K=np.copy(step_list)
	step_list_every_K=step_list_every_K.astype(float)
	for k in range(len(step_list)):
		step_list_every_K[k]=step_list[k]/1000.0
	ax.plot(step_list_every_K,[t[0] for t in train_accuracy_list],'--',color="blue")
	ax.plot(step_list_every_K,[t[0] for t in valid_accuracy_list],color="blue") 
	ax.plot(step_list_every_K,[t[1] for t in train_accuracy_list],'--',color="red")
	ax.plot(step_list_every_K,[t[1] for t in valid_accuracy_list],color="red") 
	ax.plot(step_list_every_K,[t[2] for t in train_accuracy_list],'--',color=(0,0.5,0))
	ax.plot(step_list_every_K,[t[2] for t in valid_accuracy_list],color=(0,0.5,0)) 	
	ax.plot([step_list_every_K[0],step_list_every_K[-1]],[0.9,0.9],color="black")
	ax.plot([step_list_every_K[0],step_list_every_K[-1]],[0.8,0.8],color="black")
	ax.plot([step_list_every_K[0],step_list_every_K[-1]],[0.7,0.7],color="black")
	ax.plot([step_list_every_K[0],step_list_every_K[-1]],[0.6,0.6],color="black")
	ax.set_xlabel('Iterations/K')
	plt.title('Train Accuracy (Dashed) and Valid Accuracy (Solid)')
	plt.ylim(0.0,1.0)
	
	ax2=plt.subplot(132)
	ax2.plot(step_list_every_K,[t[0] for t in train_loss_list],'--',color="blue")
	ax2.plot(step_list_every_K,[t[0] for t in valid_loss_list],color="blue") 
	ax2.plot(step_list_every_K,[t[1] for t in train_loss_list],'--',color="red")
	ax2.plot(step_list_every_K,[t[1] for t in valid_loss_list],color="red") 
	ax2.plot(step_list_every_K,[t[2] for t in train_loss_list],'--',color=(0,0.5,0))
	ax2.plot(step_list_every_K,[t[2] for t in valid_loss_list],color=(0,0.5,0)) 	
	ax2.set_xlabel('Iterations/K')
	plt.title('Train Loss (Dashed) and Valid Loss (Solid)')
	
	ax3=plt.subplot(133)
	ax3.plot(step_list_every_K,[t[0] for t in train_tendency_list],'--',color="blue")
	ax3.plot(step_list_every_K,[t[0] for t in valid_tendency_list],color="blue") 
	ax3.plot(step_list_every_K,[t[1] for t in train_tendency_list],'--',color="red")
	ax3.plot(step_list_every_K,[t[1] for t in valid_tendency_list],color="red") 
	ax3.plot(step_list_every_K,[t[2] for t in train_tendency_list],'--',color=(0,0.5,0))
	ax3.plot(step_list_every_K,[t[2] for t in valid_tendency_list],color=(0,0.5,0)) 	
	ax3.set_xlabel('Iterations/K')
	plt.title('Train Tendency (Dashed) and Valid Tendency (Solid)')
	plt.ylim(-1.0,1.0)
	
	savefig("Models/loss_accuracy_%s.png"%get_time_str()) 
	plt.close(fig)
	
	# save the data text in the file accuracy_arr.dat
	f = open('Models/loss_accuracy_list.dat', 'w+')
	f.write('%d\r\n'%(iter_times_last+ITER_TIMES))
	for i in range(len(step_list)):
		f.write("%d  %g  %g  %g  %g  %g  %g  %g  %g  %g  %g  %g  %g  %g  %g  %g  %g  %g  %g\r\n" %(step_list[i], train_loss_list[i][0], train_loss_list[i][1], train_loss_list[i][2], valid_loss_list[i][0], valid_loss_list[i][1], valid_loss_list[i][2], train_accuracy_list[i][0], train_accuracy_list[i][1], train_accuracy_list[i][2], valid_accuracy_list[i][0], valid_accuracy_list[i][1], valid_accuracy_list[i][2], train_tendency_list[i][0], train_tendency_list[i][1], train_tendency_list[i][2], valid_tendency_list[i][0], valid_tendency_list[i][1], valid_tendency_list[i][2]))
	f.close()
	shutil.copyfile('Models/loss_accuracy_list.dat', 'Models/loss_accuracy_list_%s.dat'%get_time_str())
	
	# save trained model
	if (ITER_TIMES + iter_times_last) % ITER_TIMES_PER_SAVE != 0:
		saver.save(sess, 'Models/model_%s_%d_%s.dat'%(get_time_str(),ITER_TIMES+iter_times_last,MODEL_NAME))
	saver.save(sess, 'Models/model.dat')	
