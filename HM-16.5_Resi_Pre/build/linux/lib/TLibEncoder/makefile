# the SOURCE definiton lets move your makefile to another position
CONFIG 			= LIBRARY

# set pathes to the correct directories
SRC_DIR			= ../../../../source/Lib/TLibEncoder
INC_DIR			= ../../../../source/Lib
LIB_DIR			= ../../../../lib
BIN_DIR			= ../../../../bin

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
OBJ_DIR			= ./objects$(HBD)

# the library name
PRJ_NAME	 	= TLibEncoder$(HBD)

# version information
MAJOR_VER		= 0
MINOR_VER		= 1
VER				= $(MAJOR_VER).$(MINOR_VER)

# defines to set
DEFS			= -DMSYS_LINUX

# set objects
OBJS          	= \
			$(OBJ_DIR)/NALwrite.o \
			$(OBJ_DIR)/SEIEncoder.o \
			$(OBJ_DIR)/SEIwrite.o \
			$(OBJ_DIR)/SyntaxElementWriter.o \
			$(OBJ_DIR)/TEncBinCoderCABAC.o \
			$(OBJ_DIR)/TEncBinCoderCABACCounter.o \
			$(OBJ_DIR)/TEncSampleAdaptiveOffset.o \
			$(OBJ_DIR)/TEncCavlc.o \
			$(OBJ_DIR)/TEncCu.o \
			$(OBJ_DIR)/TEncEntropy.o \
			$(OBJ_DIR)/TEncGOP.o \
			$(OBJ_DIR)/TEncSbac.o \
			$(OBJ_DIR)/TEncSearch.o \
			$(OBJ_DIR)/TEncSlice.o \
			$(OBJ_DIR)/TEncTop.o \
			$(OBJ_DIR)/TEncPic.o \
			$(OBJ_DIR)/TEncPreanalyzer.o \
			$(OBJ_DIR)/WeightPredAnalysis.o \
			$(OBJ_DIR)/TEncRateCtrl.o \

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
