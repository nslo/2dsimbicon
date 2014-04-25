SRCS = \
		 main.cpp \
		 control/simbicon_control.cpp \
		 render/drawstuff.cpp \
		 render/renderer.cpp \
		 render/x11.cpp \
		 sim/simulator.cpp \
		 sim/biped7.cpp

CC=clang++
CFLAGS = -Wall -Wextra -Werror -pedantic -std=c++11 -O2 \
	   -I$(HOME)/include -I/usr/include -I"./$(TOP_OBJ_DIR)" -I"./$(SRC_DIR)"
LDFLAGS= 
LIBS= -L/usr/lib -lGL -lGLU -lGLEW \
	-L/usr/X11R6/lib -lX11 -lXext -lm \
	-L$(HOME)/lib -lode -lgsl -lgslcblas 
			
EXECUTABLE = main
SRC_DIR = src
TOP_OBJ_DIR = bin
OBJ_DIR = $(TOP_OBJ_DIR)/$(SUB_OBJ_DIR)
OBJS = $(SRCS:.cpp=.o)
OBJS_PATH = $(addprefix $(OBJ_DIR)/,$(OBJS))
# .d files
DEPS = $(OBJS:.o=.d)
DEPS_PATH = $(addprefix $(OBJ_DIR)/,$(DEPS))
#OBJECTS=$(SOURCES:.cpp=.o)


all: $(EXECUTABLE)

$(EXECUTABLE): $(OBJS_PATH)
	$(CC) $(CFLAGS) -o $@ $^ $(LIBS)

$(OBJ_DIR)/%.d: $(SRC_DIR)/%.cpp
	mkdir -p $(@D)
	$(CC) $(CFLAGS) -MM -MP -MT $(@:.d=.o) -o $@ $<

# don't need to mkdir for object files since d files already exist
$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp
	$(CC) $(CFLAGS) -c -o $@ $<

ifneq ($(MAKECMDGOALS),clean)
-include $(DEPS_PATH)
endif

clean:
	rm -rf $(TOP_OBJ_DIR) $(EXECUTABLE)

fmt:
	astyle --recursive --style=allman --unpad-paren --pad-header --pad-oper "*.cpp"
	astyle --recursive --style=allman --unpad-paren --pad-header --pad-oper "*.h"
