cmd_drivers/spi/built-in.o :=  arm-linux-gnueabihf-ld -EL    -r -o drivers/spi/built-in.o drivers/spi/spi.o drivers/spi/spidev.o drivers/spi/spi-bitbang.o drivers/spi/spi-gpio.o drivers/spi/spi-s3c64xx.o ; scripts/mod/modpost drivers/spi/built-in.o
