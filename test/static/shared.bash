g++ -c -fPIC hieplib.cpp -o hieplib.o
g++ -shared hieplib.o -o libhiep.so
g++ hiep.cpp -o main -L. -lhiep
export LD_LIBRARY_PATH=/home/hiep/Desktop/test:$LD_LIBRARY_PATH
