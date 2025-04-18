CXX = g++  
CXXFLAGS = -std=c++20 -Wall -Wextra -O2
# CXXFLAGS = -std=c++20 -Wall -Wextra -Os -flto -s


TARGET = build/main  

SRCS = main.cpp

all: $(TARGET)

$(TARGET): $(SRCS)
	@mkdir -p build
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(SRCS)

clean:
	rm -rf build
