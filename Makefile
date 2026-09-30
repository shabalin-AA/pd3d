deps:
	mkdir -p deps
	wget https://github.com/raysan5/raylib/releases/download/6.0/raylib-6.0_linux_amd64.tar.gz
	mv raylib-6.0_linux_amd64.tar.gz deps/raylib.tar.gz
	cd deps && tar -xf raylib.tar.gz
	rm deps/raylib.tar.gz
	wget https://github.com/zpl-c/enet/releases/download/v2.7.0/enet.h
	mv enet.h deps/enet.h

build: deps
	mkdir -p bin
	gcc src/main.c -o bin/main -I./deps -I./deps/raylib-6.0_linux_amd64/include -L./deps/raylib-6.0_linux_amd64/lib -Wl,-Bstatic -lraylib -Wl,-Bdynamic -lm -lX11

run: build
	./bin/main
