cmd_drivers/net/built-in.o :=  arm-linux-gnueabihf-ld -EL    -r -o drivers/net/built-in.o drivers/net/Space.o drivers/net/loopback.o drivers/net/netconsole.o drivers/net/ethernet/built-in.o drivers/net/wireless/built-in.o ; scripts/mod/modpost drivers/net/built-in.o
