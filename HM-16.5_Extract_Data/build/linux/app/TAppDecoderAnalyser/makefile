# the SOURCE definiton lets you move your makefile to another position
CONFIG 				= CONSOLE

# set directories to your wanted values
SRC_DIR				= ../../../../source/App/TAppDecoder
INC_DIR				= ../../../../source/Lib
LIB_DIR				= ../../../../lib
BIN_DIR				= ../../../../bin

SRC_DIR1		=
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
OBJ_DIR				= ./objects$(HBD)

# set executable name
PRJ_NAME			= TAppDecoderAnalyser$(HBD)

# defines to set
DEFS				= -DMSYS_LINUX -D_LARGEFILE64_SOURCE -D_FILE_OFFSET_BITS=64 -DMSYS_UNIX_LARGEFILE -DRExt__DECODER_DEBUG_BIT_STATISTICS=1

# set objects
OBJS          		= 	\
					$(OBJ_DIR)/decmain.o \
					$(OBJ_DIR)/TAppDecCfg.o \
					$(OBJ_DIR)/TAppDecTop.o \

# set libs to link with
LIBS				= -ldl

DEBUG_LIBS			=
RELEASE_LIBS		=

STAT_LIBS			= -lpthread
DYN_LIBS			=


DYN_DEBUG_LIBS		= -lTLibDecoderAnalyser$(HBD)d -lTLibCommon$(HBD)d -lTLibVideoIO$(HBD)d -lTAppCommon$(HBD)d
DYN_DEBUG_PREREQS		= $(LIB_DIR)/libTLibDecoderAnalyser$(HBD)d.a $(LIB_DIR)/libTLibCommon$(HBD)d.a $(LIB_DIR)/libTLibVideoIO$(HBD)d.a $(LIB_DIR)/libTAppCommon$(HBD)d.a
STAT_DEBUG_LIBS		= -lTLibDecoderAnalyser$(HBD)Staticd -lTLibCommon$(HBD)Staticd -lTLibVideoIO$(HBD)Staticd -lTAppCommon$(HBD)Staticd
STAT_DEBUG_PREREQS		= $(LIB_DIR)/libTLibDecoderAnalyser$(HBD)Staticd.a $(LIB_DIR)/libTLibCommon$(HBD)Staticd.a $(LIB_DIR)/libTLibVideoIO$(HBD)Staticd.a $(LIB_DIR)/libTAppCommon$(HBD)Staticd.a

DYN_RELEASE_LIBS	= -lTLibDecoderAnalyser$(HBD) -lTLibCommon$(HBD) -lTLibVideoIO$(HBD) -lTAppCommon$(HBD)
DYN_RELEASE_PREREQS	= $(LIB_DIR)/libTLibDecoderAnalyser$(HBD).a $(LIB_DIR)/libTLibCommon$(HBD).a $(LIB_DIR)/libTLibVideoIO$(HBD).a $(LIB_DIR)/libTAppCommon$(HBD).a
STAT_RELEASE_LIBS	= -lTLibDecoderAnalyser$(HBD)Static -lTLibCommon$(HBD)Static -lTLibVideoIO$(HBD)Static -lTAppCommon$(HBD)Static
STAT_RELEASE_PREREQS	= $(LIB_DIR)/libTLibDecoderAnalyser$(HBD)Static.a $(LIB_DIR)/libTLibCommon$(HBD)Static.a $(LIB_DIR)/libTLibVideoIO$(HBD)Static.a $(LIB_DIR)/libTAppCommon$(HBD)Static.a


# name of the base makefile
MAKE_FILE_NAME		= ../../common/makefile.base

# include the base makefile
include $(MAKE_FILE_NAME)
