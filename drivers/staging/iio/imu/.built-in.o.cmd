cmd_drivers/staging/iio/imu/built-in.o :=  arm-linux-gnueabihf-ld -EL    -r -o drivers/staging/iio/imu/built-in.o drivers/staging/iio/imu/adis16400.o ; scripts/mod/modpost drivers/staging/iio/imu/built-in.o
