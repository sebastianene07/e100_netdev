all:
	make -C /home/sene/repos/linux M=$(PWD) modules

clean:
	rm -f *.ko *.o
