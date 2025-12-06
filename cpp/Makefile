CXX = g++
CXXFLAGS = -Wall -Wextra -std=c++20
TARGET = order_planner
SRCS = main.cpp utils.cpp planner.cpp
HDRS = utils.h planner.h
OBJS = $(SRCS:.cpp=.o)

# Build the executable
$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(OBJS)

# Rule for compiling .cpp files into .o files
%.o: %.cpp $(HDRS)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Clean up build files
clean:
	rm -f $(OBJS) $(TARGET)

# Phony targets
.PHONY: clean
