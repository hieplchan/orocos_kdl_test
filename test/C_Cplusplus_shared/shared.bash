g++ -c -fPIC hieplib.cpp -o hieplib.o
g++ -shared hieplib.o -o libhiep.so
gcc hiep.c -o main -L. -lhiep
export LD_LIBRARY_PATH=/home/hiep/Desktop/test/C_Cplusplus_shared:$LD_LIBRARY_PATH
