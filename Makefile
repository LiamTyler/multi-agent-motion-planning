# Root directory and structure
MAINDIR = $(realpath ./)
BUILDDIR = $(MAINDIR)/build
SRCDIR = $(MAINDIR)/src
BINDIR = $(BUILDDIR)/bin
OBJDIR = $(BUILDDIR)/obj
EXTDIR = $(MAINDIR)/ext
# Where g++ will look for header files
INCLUDEDIRS = $(MAINDIR) $(EXTDIR)

################################################################################
# Adding in Progression
#
# Root directory of Progression
PROGRESSION_ROOT = $(EXTDIR)/Progression
# Path to the folder where libProgression.a is stored
LIBDIR = $(PROGRESSION_ROOT)/build
# Path to the libProgression.a file
LIB = $(LIBDIR)/libProgression.a
# Commands for g++ to link it in
LIBLINK = -L$(LIBDIR) -lProgression
# Add root dir to the include list
INCLUDEDIRS += $(PROGRESSION_ROOT)

# last step to use progression library: Add LIB target as seen below, and add as
# dependency for the final TARGET
################################################################################

# Compiler
CXX = g++
# Compiler libraries to link
CXXLIBS += $(LIBLINK) -lGLEW -lSDL2 -lGL -lGLU -ldl
# Compiler flags. (the patsubst puts the '-I' infront of all of the directories
CXXFLAGS += $(patsubst %, -I%, $(INCLUDEDIRS)) -std=c++11 -O3 -fopenmp

# Function to get list of all files with a given extenstion. Ex: list of all cpp files
rwildcard=$(foreach d,$(wildcard $1*),$(call rwildcard,$d/,$2) $(filter $(subst *,%,$2),$d))

# Function to generate the header file dependencies while compiling
make-depend-cxx=$(CXX) $(CXXFLAGS) -MM -MF $3 -MP -MT $2 $1

# Create list of all .cpp files
SRC_CXX = $(call rwildcard,$(SRCDIR),*.cpp)
# Create list of all .o files to create
OBJECTS_CXX = $(notdir $(patsubst %.cpp,%.o,$(SRC_CXX)))

# Final executable name
TARGET = $(BINDIR)/proj

.PHONY: all clean veryclean run

all: $(TARGET)

$(TARGET): $(LIB) $(addprefix $(OBJDIR)/, $(OBJECTS_CXX)) | $(BINDIR)
	$(CXX) $(CXXFLAGS) $(addprefix $(OBJDIR)/, $(OBJECTS_CXX)) $(LIBLINK) $(CXXLIBS) -o $@

$(LIB):
	cd $(PROGRESSION_ROOT) && $(MAKE) lib

clean:
	@rm -rf $(BUILDDIR)

veryclean:
	@rm -rf $(BUILDDIR)
	@rm -rf $(LIBDIR)

run: $(TARGET)
	$(TARGET)

ifneq "$MAKECMDGOALS" "clean"
-include $(addprefix $(OBJDIR)/,$(OBJECTS_CXX:.o=.d))
endif

$(addprefix $(OBJDIR)/, $(OBJECTS_CXX)): | $(OBJDIR)

$(BINDIR) $(OBJDIR):
	@mkdir -p $@

$(OBJDIR)/%.o: $(SRCDIR)/%.cpp
	@$(call make-depend-cxx,$<,$@,$(subst .o,.d,$@))
	$(CXX) $(CXXFLAGS) $(CXXLIBS) -c -o $@ $<
