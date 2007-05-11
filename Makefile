CFLAGS = -g3 -O2 -Wall -fPIC

libsane-ls5000.so.1: ls5000.o sanei_usb.o debug.o config.h
	$(CC) -lsane -lusb -shared -o $@ -Wl,-soname,$@ $^

*.o:	config.h

clean:
	rm -f libsane-ls5000.so.1 *.o *~

re:	clean libsane-ls5000.so.1
