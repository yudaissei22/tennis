
all: eus_shm_interface.so # test_eus_shm

eus_shm_interface.so: eus_shm_interface.c
	gcc -fPIC  -c eus_shm_interface.c
	gcc -fPIC  -shared -Wl,-soname,eus_shm_interface.so -o eus_shm_interface.so eus_shm_interface.o

clean:
	rm -rf *.so *.o
