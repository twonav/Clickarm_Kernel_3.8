cmd_drivers/media/mmc/built-in.o :=  arm-linux-gnueabihf-ld -EL    -r -o drivers/media/mmc/built-in.o drivers/media/mmc/siano/built-in.o ; scripts/mod/modpost drivers/media/mmc/built-in.o
