cmd_drivers/i2c/algos/built-in.o :=  arm-linux-gnueabihf-ld -EL    -r -o drivers/i2c/algos/built-in.o drivers/i2c/algos/i2c-algo-bit.o ; scripts/mod/modpost drivers/i2c/algos/built-in.o
