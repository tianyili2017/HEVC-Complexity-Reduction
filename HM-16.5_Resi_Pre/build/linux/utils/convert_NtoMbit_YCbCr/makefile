# the SOURCE definiton lets you move your makefile to another position
CONFIG 				= CONSOLE

# set directories to your wanted values
SRC_DIR				= ../../../../source/App/utils
INC_DIR				= ../../../../source/Lib
LIB_DIR				= ../../../../lib
BIN_DIR				= ../../../../bin

SRC_DIR1		=
SRC_DIR2		=
SRC_DIR3		=
SRC_DIR4		=

USER_INC_DIRS	= -I$(SRC_DIR) 
USER_LIB_DIRS	=

# intermediate directory for object files
OBJ_DIR				= ./objects

# set executable name
PRJ_NAME			= convert_NtoMbit_YCbCr

# defines to set
DEFS				= -DMSYS_LINUX -D_LARGEFILE64_SOURCE -D_FILE_OFFSET_BITS=64 -DMSYS_UNIX_LARGEFILE

# set objects
OBJS          		= 	\
					$(OBJ_DIR)/convert_NtoMbit_YCbCr.o \

# set libs to link with
LIBS				= -ldl

DEBUG_LIBS			=
RELEASE_LIBS		=

STAT_LIBS			= -lpthread
DYN_LIBS			=


DYN_DEBUG_LIBS		= -lTLibDecoderd -lTLibCommond -lTLibVideoIOd -lTAppCommond
DYN_DEBUG_PREREQS		= $(LIB_DIR)/libTLibDecoderd.a $(LIB_DIR)/libTLibCommond.a $(LIB_DIR)/libTLibVideoIOd.a $(LIB_DIR)/libTAppCommond.a
STAT_DEBUG_LIBS		= -lTLibDecoderStaticd -lTLibCommonStaticd -lTLibVideoIOStaticd -lTAppCommonStaticd
STAT_DEBUG_PREREQS		= $(LIB_DIR)/libTLibDecoderStaticd.a $(LIB_DIR)/libTLibCommonStaticd.a $(LIB_DIR)/libTLibVideoIOStaticd.a $(LIB_DIR)/libTAppCommonStaticd.a

DYN_RELEASE_LIBS	= -lTLibDecoder -lTLibCommon -lTLibVideoIO -lTAppCommon
DYN_RELEASE_PREREQS	= $(LIB_DIR)/libTLibDecoder.a $(LIB_DIR)/libTLibCommon.a $(LIB_DIR)/libTLibVideoIO.a $(LIB_DIR)/libTAppCommon.a
STAT_RELEASE_LIBS	= -lTLibDecoderStatic -lTLibCommonStatic -lTLibVideoIOStatic -lTAppCommonStatic
STAT_RELEASE_PREREQS	= $(LIB_DIR)/libTLibDecoderStatic.a $(LIB_DIR)/libTLibCommonStatic.a $(LIB_DIR)/libTLibVideoIOStatic.a $(LIB_DIR)/libTAppCommonStatic.a


# name of the base makefile
MAKE_FILE_NAME		= ../../common/makefile.base

# include the base makefile
include $(MAKE_FILE_NAME)
