cmd_drivers/power/built-in.o :=  arm-linux-gnueabihf-ld -EL    -r -o drivers/power/built-in.o drivers/power/power_supply.o drivers/power/ds2782_battery.o drivers/power/reset/built-in.o ; scripts/mod/modpost drivers/power/built-in.o
