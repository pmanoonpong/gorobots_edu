# generic makefile for simulations and other projects inside the gorobots
# repository
#
# You should include this Makefile inside the local Makefile of your project.
# Before including, you need to fill the FILE variable with the source files
# belonging to your project and set the GOROBOTS variable to point to the root
# gorobots root directory. For instance:
#
#  FILES    += main
#  GOROBOTS = ../../..
#  include $(GOROBOTS)/simulation.makefile



# filename of simulation executables
EXEC = start

# the pure filename of the gorobots library (withoug lib and .a)
GOROBOTS_LIBNAME = gorobots

# deduct names of source, object and dependency files
CFILES = $(addsuffix .cpp, $(FILES))
OFILES = $(addsuffix .o, $(FILES))
DFILES = $(addsuffix .d, $(FILES))

# libraries needed to link with ode_robot
ODE_ROBOTS_LIBS = $(shell ode_robots-config --static --libs)
# libraries needed to link with selforg
SELFORG_LIBS    = $(shell selforg-config --static --libs)
# libraries needed to link with gorobots
GOROBOTS_LIBS   = -L$(GOROBOTS) \
                  -Wl,-Bstatic -l$(GOROBOTS_LIBNAME)

# source location of ode_robots
ODEROBOTS_SRCPREFIX = $(shell ode_robots-config $(CFGOPTS) --srcprefix)
# static library file of ode_robots
LIBFILE_ODEROBOTS  = $(shell ode_robots-config $(CFGOPTS) --libfile)
# static library file of selforg
LIBFILE_SELFORG    = $(shell selforg-config $(CFGOPTS) --libfile)
# static library file of gorobots
LIBFILE_GOROBOTS   = $(GOROBOTS)/lib$(GOROBOTS_LIBNAME).a

# Include folders
INC += -I. -I$(GOROBOTS)
# libraries needed to link everything
LIBS += $(GOROBOTS_LIBS) $(ODE_ROBOTS_LIBS) $(SELFORG_LIBS) 

# Flags for the c++ compiler:
# -Wall             : enable all warnings
# -pipe             : Use pipes rather than temporary files for communication 
#                     between the various stages of compilation
# -MMD              : Output dependency rules (but only for user header files) 
#                     while at the same time compiling as usual
# -MP               : This option instructs CPP to add a phony target for each 
#                     dependency other than the main file, causing each to 
#                     depend on nothing. These dummy rules work around errors
#                     make gives if you remove header files without updating 
#                     the Makefile to match.
# $(INC)            : include directories
# selforg-config    : configuration script for selforg
#   --cflags        : generate flags for the gcc compiler
# ode_robots_config :
#   --intern        : add debug symbols and light optimization
#   --cflags        : generate flags for the gcc compiler
CXXFLAGS = -Wall \
           -pipe \
           -MMD \
           -MP \
           $(INC) \
           $(shell selforg-config --cflags) \
           $(shell ode_robots-config --intern --cflags) 

# Target stating that a normal build includes updates of ode_robots (includes
# selforg) and gorobots and finally a build of the executable. The bar "|" 
# enforces correct order
normal: | libode_robots libgorobots $(EXEC) 

# Rule explaining how to build the executable and which files it depends on
$(EXEC): $(OFILES) $(LIBFILE_ODEROBOTS) $(LIBFILE_SELFORG) $(LIBFILE_GOROBOTS)
	$(CXX) $(CXXFLAGS) $(OFILES) $(LIBS) -o $(EXEC)

# Rule for updating the ode_robots library
libode_robots:
	cd $(ODEROBOTS_SRCPREFIX) && $(MAKE) lib

# Rule for updating the gorobots library
libgorobots:
	cd $(GOROBOTS) && $(MAKE) statlib

# remove all built files
clean:
	rm -f $(EXEC) $(OFILES) $(DFILES)

# include the automatically created dependencies
-include $(DFILES)
