all:
	g++ `pkg-config --cflags opencv` `pkg-config --libs opencv`-lwiringPi -pthread -o newline LineDetectorNew.cpp
	sudo ./newline
