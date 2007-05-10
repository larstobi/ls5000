CFLAGS=-g3 -O2 -Wall -fPIC

ls5000.so: ls5000.o sanei_usb.o debug.o config.h
	$(CC) -shared -o $@ -Wl,-soname,$@ -lc $^

*.o:	config.h

clean:
	rm -f *.so *.o *~

re:	clean ls5000.so
