# the SOURCE definiton lets move your makefile to another position
CONFIG 			= LIBRARY

# set pathes to the correct directories
SRC_DIR			= ../../../../source/Lib/TLibDecoder
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
PRJ_NAME	 	= TLibDecoderAnalyser$(HBD)

# version information
MAJOR_VER		= 0
MINOR_VER		= 1
VER				= $(MAJOR_VER).$(MINOR_VER)

# defines to set
DEFS			= -DMSYS_LINUX -DRExt__DECODER_DEBUG_BIT_STATISTICS=1

# set objects
OBJS          	= \
				$(OBJ_DIR)/AnnexBread.o \
				$(OBJ_DIR)/NALread.o \
				$(OBJ_DIR)/SEIread.o \
				$(OBJ_DIR)/SyntaxElementParser.o \
				$(OBJ_DIR)/TDecBinCoderCABAC.o \
				$(OBJ_DIR)/TDecCAVLC.o \
				$(OBJ_DIR)/TDecCu.o \
				$(OBJ_DIR)/TDecEntropy.o \
				$(OBJ_DIR)/TDecGop.o \
				$(OBJ_DIR)/TDecSbac.o \
				$(OBJ_DIR)/TDecSlice.o \
				$(OBJ_DIR)/TDecTop.o \

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
