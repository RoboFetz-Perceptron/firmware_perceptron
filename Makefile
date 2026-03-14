PORT   ?= /dev/ttyACM0
BAUD   ?= 460800

.PHONY: build flash monitor flash-monitor clean fullclean menuconfig

build:
	idf.py build

flash: build
	idf.py -p $(PORT) -b $(BAUD) flash

monitor:
	idf.py -p $(PORT) monitor

flash-monitor: build
	idf.py -p $(PORT) -b $(BAUD) flash monitor

menuconfig:
	idf.py menuconfig

clean:
	idf.py clean

fullclean:
	idf.py fullclean
