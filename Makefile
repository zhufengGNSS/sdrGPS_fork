#
# Makefile for software-radio GPS project
# by Yu lu, luyu1974@gmail.com
#  Dec,2004
#



ifdef DBG
DBG_RELEASE = -O3 -fomit-frame-pointer
DBG_DEBUG = -O -gdwarf-2
DEBUG=$(DBG_$(DBG))
endif

ifndef DEBUG
DEBUG = -g
endif

ifndef OPTIM
OPTIM = -O3 -D__INLINE_FUNC__  -DGPS_LOG_FILE
endif


OBJ_PATH = ./obj/
LIB_OBJ_PATH = ./libobj/
SRC_PATH = ./src/
LIB_PATH = ./src/lib/
HDR_PATH = ./src/includes/
LIB_HDR_PATH = ./src/lib/includes/
LOG_PATH = ./data/

CC  = g++
CXX = g++
LD  = g++


CFLAGS  = -Wall $(OPTIM) $(DEBUG)  
CXXFLAGS = $(CFLAGS)
APPCXXFLAGS = $(CFLAGS) -lpanel -lncurses -lpthread

EXECUTABLE = sdrGPS
SOFTGPSLIB = libsoftrcvr.a

SRC = main.cc sig_source.cc configparser.cc display_win.cc

LIB_SRC = sinu_nco.cc square_nco.cc prncode_generator.cc cacode_module.cc \
onech_correlator.cc   gps_controller.cc acquisition.cc \
gps_nav_fix.cc multich_correlator.cc gps_nav_msg.cc pos_math.cc matrix.cc kalman_filter.cc softrcvr.cc

MYSRC = $(addprefix $(SRC_PATH), $(SRC)) 
MYOBJ = $(addprefix $(OBJ_PATH), $(addsuffix .o, $(basename $(SRC))))

LIBSRC = $(addprefix $(LIB_PATH), $(LIB_SRC)) 
LIBOBJ = $(addprefix $(LIB_OBJ_PATH), $(addsuffix .o, $(basename $(LIB_SRC))))

#MYOBJ = $(OBJ_PATH)main.o $(OBJ_PATH)sinu_nco.o
MYHDR = $(addprefix $(HDR_PATH), $(addsuffix .h, $(basename $(SRC))))
LIBHDR = $(addprefix $(LIB_HDR_PATH), $(addsuffic .h $(basename $LIB_SRC)))

LDFLAGS = -V$(PLATFORM) -Bstatic $(DEBUG) 
SDFLAGS = -V$(PLATFORM)          $(DEBUG) 

VPATH = ./

#--------------suffix rules ---------------------
# set up c++ suffixes and relationship between .cc and .o files
.SUFFIXES: .cc .o .h
.cc.o:
	$(CC) $(CXXFLAGS) -c $<

COMPILE = $(CC) $(CXXFLAGS) -c
APPCOMPILE = $(CC) $(APPCXXFLAGS) -c

$(OBJ_PATH)%.o: $(SRC_PATH)%.cc 
	$(APPCOMPILE) -o $@ $<
$(LIB_OBJ_PATH)%.o:$(LIB_PATH)%.cc
	$(COMPILE) -o $@ $<

#
# Application 


$(EXECUTABLE): $(SOFTGPSLIB) $(MYOBJ)
	$(CC) $(APPCXXFLAGS) -o $@ $(MYOBJ) $(SOFTGPSLIB)

$(SOFTGPSLIB):$(LIBOBJ)
	@echo 
	@echo "****************************************"
	@echo "Begining of building softgps lib        "
	@echo "****************************************"
	ar rs  $(SOFTGPSLIB) $(LIBOBJ)
	@echo "****************************************"
	@echo "End of building softgps lib             "
	@echo "****************************************"
#
# Headers
#

$(MYOBJ):$(MYHDR)
$(LIBOBJ):$(LIBHDR)

# clean up extreaneous files
clean:
	-rm -rf  $(EXECUTABLE) *~ *# $(SRC_PATH)*# $(SRC_PATH)*~ \
         $(HDR_PATH)*# $(HDR_PATH)*~ $(LIB_PATH)*# $(LIB_PATH)*~ \
	$(LIB_HDR_PATH)*# $(LIB_HDR_PATH)*~ $(OBJ_PATH)*.o $(EXECUTABLE).tar. \
	$(LOG_PATH)*.m  $(LIB_OBJ_PATH)*.o $(SOFTGPSLIB) 

# generate a debugging version of the code
debug:
	DEBUG=yes
	make
homear:
	rm -rf ./sdrGPS.tar
	rm -rf ./sdrGPS.tar.gz
	rm -rf ./sdrGPS
	mkdir ./sdrGPS
	mkdir ./sdrGPS/data
	mkdir ./sdrGPS/doc
	mkdir ./sdrGPS/obj
	mkdir ./sdrGPS/libobj
	mkdir ./sdrGPS/src
	mkdir ./sdrGPS/scilab
	mkdir ./sdrGPS/src/includes
	mkdir ./sdrGPS/IF_data
	-cp ./* ./sdrGPS
	-cp ./data/hexeph.bin ./sdrGPS/data
	-cp -r ./src/* ./sdrGPS/src
	-cp ./src/includes/* ./sdrGPS/src/includes
	-cp ./doc/* ./sdrGPS/doc
	-cp ./scilab/* ./sdrGPS/scilab
	-cp ./IF_data/* ./sdrGPS/IF_data
	tar -cvf sdrGPS.tar ./sdrGPS
	gzip sdrGPS.tar
	rm -rf ./sdrGPS
archive: homear
	scp sdrGPS.tar.gz ylu@storm.ee.ucr.edu:~/my_file
