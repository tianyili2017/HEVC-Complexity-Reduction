import os
import glob
import random
import numpy as np
import data_info as di

# ----------------------------------------------------------------------------------------------------------------------
# To configure: three variables (CONFIG, YUV_PATH_RESI, INFO_PATH)
CONFIG = 'LDP' # coding configuration for HEVC
if CONFIG == 'LDP': # Low-Delay-P
    YUV_PATH_RESI = '/media/F/DataHEVC/LDP_Resi_Pre/' # path storing resi_XX.yuv files
    INFO_PATH = '/media/F/DataHEVC/LDP_Info/' # path storing Info_XX.dat files
elif CONFIG == 'LDB': # Low-Delay-B
    YUV_PATH_RESI = '/media/F/DataHEVC/LDB_Resi_Pre/'
    INFO_PATH = '/media/F/DataHEVC/LDB_Info/'
elif CONFIG == 'RA': # Random-Access. Here the loaded data are organized in displaying order. However, the generated will be stored in encoding order finally.
    YUV_PATH_RESI = '/media/F/DataHEVC/RA_Resi_Pre/'
    INFO_PATH = '/media/F/DataHEVC/RA_Info/'
# ----------------------------------------------------------------------------------------------------------------------

YUV_NAME_LIST_FULL = di.YUV_NAME_LIST_FULL
YUV_WIDTH_LIST_FULL = di.YUV_WIDTH_LIST_FULL
YUV_HEIGHT_LIST_FULL = di.YUV_HEIGHT_LIST_FULL

QP_LIST = [22, 27, 32, 37]

INDEX_LIST_VALID = [36,42,64,66,72,74,92,97,101,110]
INDEX_LIST_TRAIN = [v for v in list(range(30, 123)) if v not in INDEX_LIST_VALID]
INDEX_LIST_TEST = list(range(12, 30))

def get_file_list(yuv_path_resi, info_path, yuv_name_list_full, qp_list, index_list):

    yuv_name_list = [yuv_name_list_full[index] for index in index_list]
    yuv_file_list = []
    info_file_list = []

    for i_qp in range(len(qp_list)):
        yuv_file_list.append([])
        info_file_list.append([])
        for i_seq in range(len(index_list)):
            yuv_file_list[i_qp].append([])
            info_file_list[i_qp].append([])

    for i_seq in range(len(index_list)):
        for i_qp in range(len(qp_list)):
            yuv_file_temp = glob.glob(yuv_path_resi + 'resi*_' + yuv_name_list[i_seq] + '_*qp' + str(qp_list[i_qp]) + '*.yuv')
            assert(len(yuv_file_temp) == 1)
            yuv_file_list[i_qp][i_seq] = yuv_file_temp[0]
            info_file_temp = glob.glob(info_path + 'Info*_' + yuv_name_list[i_seq] + '_*qp' + str(qp_list[i_qp]) + '*CUDepth.dat')
            assert(len(info_file_temp) == 1)
            info_file_list[i_qp][i_seq] = info_file_temp[0]

    return yuv_file_list, info_file_list

class FrameYUV(object):
    def __init__(self, Y, U, V):
        self._Y = Y
        self._U = U
        self._V = V

def get_file_size(path):
    try:
        size = os.path.getsize(path)
        return size
    except Exception as err:
        print(err)

def encode_to_display_order(config, i_frame, num_frame):
    if config == 'RA':
        if i_frame == 0:
            return 0
        else:
            table = [7,3,1,0,2,5,4,6]
            i_GOP = (i_frame - 1) // 8
            i_frame_in_GOP = (i_frame - 1) % 8
            GOP_length = min(num_frame - 1 - i_GOP * 8, 8)
            if GOP_length < 8:
                table = [x for x in table if x < GOP_length]
            i_frame_in_GOP = table[i_frame_in_GOP]
            return 1 + i_frame_in_GOP + i_GOP * 8
    else:
        return i_frame

def get_num_YUV420_frame(file, width, height):
    file_bytes = get_file_size(file)
    frame_bytes = width * height * 3 // 2
    print(file_bytes, frame_bytes)
    assert(file_bytes % frame_bytes == 0)
    return file_bytes // frame_bytes

def read_YUV420_frame(file, i_frame, width, height):
    # read a frame from a YUV420-formatted sequence
    d00 = height // 2
    d01 = width // 2
    fid = open(file, 'rb')
    fid.seek(width * height * i_frame * 3 // 2)
    Y_buf = fid.read(width * height)
    Y = np.reshape(np.frombuffer(Y_buf, dtype=np.uint8), [height, width])
    U_buf = fid.read(d01 * d00)
    U = np.reshape(np.frombuffer(U_buf, dtype=np.uint8), [d00, d01])
    V_buf = fid.read(d01 * d00)
    V = np.reshape(np.frombuffer(V_buf, dtype=np.uint8), [d00, d01])
    fid.close()
    return FrameYUV(Y, U, V)

def read_info_frame(file, i_frame, width, height, mode):
    # read information of CU/TU partition
    assert(width % 8 == 0 and height % 8 == 0)
    if mode == 'CU':
        unit_width = 16
    elif mode == 'TU':
        unit_width = 8
    num_line_in_unit = height // unit_width
    num_column_in_unit = width // unit_width
    fid = open(file, 'rb')
    fid.seek(num_column_in_unit * num_line_in_unit * i_frame)
    info_buf = fid.read(num_line_in_unit * num_column_in_unit)
    info = np.reshape(np.frombuffer(info_buf, dtype = np.uint8), [num_line_in_unit, num_column_in_unit])
    fid.close()
    return info

def write_data(fid_out, i_seq, i_frame, qp_list, frame_Y_list, cu_depth_mat_list):
    width = np.shape(frame_Y_list[0])[1]
    height = np.shape(frame_Y_list[0])[0]
    assert(len(qp_list) == len(cu_depth_mat_list))
    n_qp = len(qp_list)
    n_line = height // 64
    n_col = width // 64
    for i_line in range(n_line):
        for i_col in range(n_col):
            buf_sample = (np.ones((64 + 4 * (1 + 16 + 4096),)) * 255).astype(np.uint8)
            buf_sample[0] = 1 # 1 byte: number of frames (always 1)
            buf_sample[2] = width % 256 # 2 bytes: frame width
            buf_sample[3] = width // 256
            buf_sample[4] = height % 256 # 2 bytes: frame height
            buf_sample[5] = height // 256
            buf_sample[10] = i_frame % 256 # 4 bytes: order of frame
            buf_sample[11] = (i_frame >> 8) % 256
            buf_sample[12] = (i_frame >> 16) % 256
            buf_sample[13] = (i_frame >> 24) % 256
            buf_sample[14] = i_line % 256 # 2 bytes: line of current patch
            buf_sample[15] = i_line // 256
            buf_sample[16] = i_col % 256 # 2 bytes: column of current patch
            buf_sample[17] = i_col // 256
            buf_sample[18] = i_seq % 256 # 2 bytes: order of sequence
            buf_sample[19] = i_seq // 256

            for i_qp in range(n_qp):
                i_start_in_buf = 64 + i_qp * (1 + 16 + 4096)
                patch_Y = frame_Y_list[i_qp][i_line * 64 : (i_line + 1) * 64, i_col * 64 : (i_col + 1) * 64]
                patch_cu_depth = cu_depth_mat_list[i_qp][i_line * 4 : (i_line + 1) * 4, i_col * 4 : (i_col + 1) * 4]
                buf_sample[i_start_in_buf] = qp_list[i_qp]
                buf_sample[i_start_in_buf + 17 : i_start_in_buf + 17 + 4096] = np.reshape(patch_Y, (4096,))
                buf_sample[i_start_in_buf + 1: i_start_in_buf + 17] = np.reshape(patch_cu_depth, (16,))
            fid_out.write(buf_sample)
    return n_line * n_col

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

def generate_data(yuv_path_resi, info_path, yuv_name_list_full,
                  yuv_width_list_full, yuv_height_list_full, qp_list, index_list, save_file, config):

    yuv_file_list, info_file_list = get_file_list(yuv_path_resi, info_path, yuv_name_list_full, qp_list, index_list)
    yuv_width_list = yuv_width_list_full[index_list]
    yuv_height_list = yuv_height_list_full[index_list]

    n_seq = len(yuv_file_list[0])
    n_qp = len(qp_list)

    fid_out = open(save_file, 'wb+')

    n_sample = 0
    for i_seq in range(n_seq):
        width = yuv_width_list[i_seq]
        height = yuv_height_list[i_seq]
        n_frame = get_num_YUV420_frame(yuv_file_list[0][i_seq], width, height)

        for i_frame_enc in range(1, n_frame): # except POC 0 (the initial I-frame)
            frame_Y_list = []
            cu_depth_mat_list = []
            i_frame_disp = encode_to_display_order(config, i_frame_enc, n_frame)
            for i_qp in range(n_qp):
                frame_YUV = read_YUV420_frame(yuv_file_list[i_qp][i_seq], i_frame_disp, width, height)
                frame_Y_list.append(frame_YUV._Y)
                cu_depth_mat_list.append(read_info_frame(info_file_list[i_qp][i_seq], i_frame_disp, width, height, 'CU'))
            n_sample_one_frame = write_data(fid_out, i_seq, i_frame_enc, qp_list, frame_Y_list, cu_depth_mat_list)
            n_sample += n_sample_one_frame
            print('Seq. %d / %d, %50s : frame %d / %d, %8d samples generated.' % (i_seq + 1, n_seq, yuv_file_list[-1][i_seq], i_frame_enc + 1, n_frame, n_sample))

    fid_out.close()

    save_file_renamed = '%s_%s_%d.dat' % (config, save_file, n_sample)
    os.rename(save_file, save_file_renamed)
    shuffle_samples(save_file_renamed, 16516)

if __name__ == '__main__':

    generate_data(YUV_PATH_RESI, INFO_PATH, YUV_NAME_LIST_FULL,
                  YUV_WIDTH_LIST_FULL, YUV_HEIGHT_LIST_FULL, QP_LIST, INDEX_LIST_TRAIN, 'Train', CONFIG)

    generate_data(YUV_PATH_RESI, INFO_PATH, YUV_NAME_LIST_FULL,
                  YUV_WIDTH_LIST_FULL, YUV_HEIGHT_LIST_FULL, QP_LIST, INDEX_LIST_VALID, 'Valid', CONFIG)

    generate_data(YUV_PATH_RESI, INFO_PATH, YUV_NAME_LIST_FULL,
                  YUV_WIDTH_LIST_FULL, YUV_HEIGHT_LIST_FULL, QP_LIST, INDEX_LIST_TEST, 'Test', CONFIG)
