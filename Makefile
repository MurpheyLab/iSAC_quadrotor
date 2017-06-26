CPP = g++
PROJ = Quad_euler
DEPS = $(PROJ).cpp
USERDIR = ./user
LIBDIR = ./src

# Prefer static libs to dynamic
LIBS = -Wl,-Bstatic \
	  	 -Wl,-Bdynamic
#	  -lrt -Wl,-Bdynamic






 
#CFLAGS = -I/home/emmanouil/eigen-eigen-07105f7124f9/ -I/home/emmanouil/boost_1_60_0 -I$(USERDIR) -I$(LIBDIR)
CFLAGS = -I"C:/Users/Manolis/Desktop/libraries/eigen-eigen-1306d75b4a21" -I"C:/Users/Manolis/Desktop/libraries/boost_1_57_0" -I$(USERDIR) -I$(LIBDIR)

# Default code optimization level
OPT = -O2

all: $(PROJ)

# Debug build with extra warning turned on and optimization off
debug: OPT = -O0 -g -m64
debug: CFLAGS += -Wall -Wextra
debug: $(PROJ)

$(PROJ): $(DEPS)
	$(CPP) $(CFLAGS) $^ -o $@ $(OPT) $(LIBS)

clean:
	rm -f *.o *.exe $(PROJ)
