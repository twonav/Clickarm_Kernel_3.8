cmd_drivers/media/usb/built-in.o :=  arm-linux-gnueabihf-ld -EL    -r -o drivers/media/usb/built-in.o drivers/media/usb/ttusb-dec/built-in.o drivers/media/usb/ttusb-budget/built-in.o drivers/media/usb/dvb-usb/built-in.o drivers/media/usb/dvb-usb-v2/built-in.o drivers/media/usb/siano/built-in.o drivers/media/usb/b2c2/built-in.o drivers/media/usb/zr364xx/built-in.o drivers/media/usb/stkwebcam/built-in.o drivers/media/usb/s2255/built-in.o ; scripts/mod/modpost drivers/media/usb/built-in.o
