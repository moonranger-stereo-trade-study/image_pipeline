HALIDE_DIR ?= ~/15618/Halide
target = basic_blur
object: $(target).cpp
	g++ $(target).cpp -g -I $(HALIDE_DIR)/include/ -I $(HALIDE_DIR)/tools/ -L $(HALIDE_DIR)/bin/ -lhalide `libpng-config --cflags --ldflags` -ljpeg -lpthread -ldl -o $(target) -std=c++11
test: object 
	LD_LIBRARY_PATH=$(HALIDE_DIR)/bin/ ./$(target)
clean:
	rm -f $(target)
	rm -f output_images/*
