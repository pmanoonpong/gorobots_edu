#Global path to gorobots directory
GOROBOTS = /home/poma/workspace/pmanoonpong-gorobots-fork/

#Source directories
SOURCE = $(GOROBOTS)utils/real_robots/sensors/xIMU/
#Include files
INC = $(addprefix -I, $(SOURCE))
#Libraries
LIBS = -lboost_thread -lboost_system -lconfig++  -lftd2xx -lstdc++ -lm

#Get all .cpp and .o files
CPP_FILES = $(wildcard $(addsuffix *.cpp, $(SOURCE)))
OBJ_FILES = $(addsuffix .o, $(basename $(CPP_FILES)))

#Compiler flags
CC_FLAGS = -O3 -Wall 

#Phony targets for cleaning
.PHONY: clean

#Prevent .o files from being automatically deleted
.PRECIOUS: '%.o'

#Get all targets by finding all .cpp files in current directory
TARGETS = $(shell for file in `find . -maxdepth 1 -name '*.cpp' -type f -printf "%f\n" | sed 's/\..*//'`; do echo "$$file "; done;)

#Make all targets
all: $(TARGETS)

#Define individual targets for each executable
#To be able to detect changes in the main .cpp file, add .cpp to prerequisites
#To do this, use second expansion
.SECONDEXPANSION:
$(TARGETS) : $(addprefix $$@,.cpp) $(OBJ_FILES)
	gcc $(CC_FLAGS) $(addprefix $@,.cpp) -o $@ $(OBJ_FILES) $(INC) $(LIBS)

#Automatic rule to generate .o files. For this get .cpp name from
#.o file name. This is done via second expansion
$(OBJ_FILES) : $$(addprefix $$(basename $$@),.cpp)
	gcc $(CC_FLAGS) -c -o $@ $< $(LIBS) $(INC)



clean:
	rm -f $(addsuffix *.o, $(SOURCE))
	find -maxdepth 1 -executable -type f -delete
	

	