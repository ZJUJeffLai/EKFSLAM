CC = g++
IDIR =/usr/include/python2.7
CFLAGS=-I$(IDIR)
LINK=-lpython2.7
STD=-std=c++11

## TODO ##
EKFSLAM: 

clean:
	rm EKFSLAM
