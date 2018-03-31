# the SOURCE definiton lets move your makefile to another position
CONFIG 			= LIBRARY

# set pathes to the correct directories
SRC_DIR			= ../../../../source/Lib/TLibCommon
INC_DIR			= ../../../../source/Lib
LIB_DIR			= ../../../../lib
BIN_DIR			= ../../../../bin

SRC_DIR1		= ../../../../source/Lib/libmd5
SRC_DIR2		=
SRC_DIR3		=
SRC_DIR4		=


USER_INC_DIRS	= -I$(SRC_DIR)
USER_LIB_DIRS	=

ifeq ($(HIGHBITDEPTH), 1)
HBD=HighBitDepth
else
HBD=
endif

# intermediate directory for object files
OBJ_DIR			= ./objects$(HBD)

# the library name
PRJ_NAME	 	= TLibCommon$(HBD)

# version information
MAJOR_VER		= 0
MINOR_VER		= 1
VER				= $(MAJOR_VER).$(MINOR_VER)

# defines to set
DEFS			= -DMSYS_LINUX

# set objects
OBJS          	= \
			$(OBJ_DIR)/Debug.o \
			$(OBJ_DIR)/TComPicYuv.o \
			$(OBJ_DIR)/TComYuv.o \
			$(OBJ_DIR)/ContextModel.o \
			$(OBJ_DIR)/ContextModel3DBuffer.o \
			$(OBJ_DIR)/SEI.o \
			$(OBJ_DIR)/TComCABACTables.o \
			$(OBJ_DIR)/TComSampleAdaptiveOffset.o \
			$(OBJ_DIR)/TComBitStream.o \
			$(OBJ_DIR)/TComChromaFormat.o \
			$(OBJ_DIR)/TComDataCU.o \
			$(OBJ_DIR)/TComLoopFilter.o \
			$(OBJ_DIR)/TComMotionInfo.o \
			$(OBJ_DIR)/TComPattern.o \
			$(OBJ_DIR)/TComPic.o \
			$(OBJ_DIR)/TComPicSym.o \
			$(OBJ_DIR)/TComPicYuvMD5.o \
			$(OBJ_DIR)/TComPrediction.o \
			$(OBJ_DIR)/TComRdCost.o \
			$(OBJ_DIR)/TComRom.o \
			$(OBJ_DIR)/TComSlice.o \
			$(OBJ_DIR)/TComTrQuant.o \
			$(OBJ_DIR)/TComTU.o \
			$(OBJ_DIR)/TComInterpolationFilter.o \
			$(OBJ_DIR)/libmd5.o \
			$(OBJ_DIR)/TComWeightPrediction.o \
			$(OBJ_DIR)/TComRdCostWeightPrediction.o \

LIBS				= -lpthread

DEBUG_LIBS			=
RELEASE_LIBS		=

STAT_LIBS			=
DYN_LIBS			= -ldl

# the libraries to link with
STAT_DEBUG_LIBS		=
STAT_RELEASE_LIBS	=
DYN_DEBUG_LIBS		=
DYN_RELEASE_LIBS	=

# name of the base makefile
MAKE_FILE_NAME		= ../../common/makefile.base

# include the base makefile
include $(MAKE_FILE_NAME)
