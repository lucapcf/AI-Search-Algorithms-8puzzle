CXX = g++  

CXXFLAGS = -std=c++17 -Wall -Wextra -O2  

TARGET = main  

SRCS = main.cpp

all: $(TARGET)
$(TARGET): $(SRCS)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(SRCS)

clean:
	rm -f $(TARGET)