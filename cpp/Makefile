CXX = g++
CXXFLAGS = -Wall -Wextra -std=c++20
TARGET = order_planner
SRCS = main.cpp utils.cpp planner.cpp
HDRS = utils.h planner.h
OBJS = $(SRCS:.cpp=.o)

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(OBJS)

%.o: %.cpp $(HDRS)
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -f $(OBJS) $(TARGET)

.PHONY: clean
