CXX = g++  
CXXFLAGS = -std=c++20 -Wall -Wextra -O2  

TARGET = main  

SRCS = main.cpp

all: $(TARGET)

$(TARGET): $(SRCS)
	@mkdir -p build
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(SRCS)

clean:
	rm -rf build
