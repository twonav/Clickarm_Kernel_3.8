cmd_fs/debugfs/debugfs.o := arm-linux-gnueabihf-ld -EL    -r -o fs/debugfs/debugfs.o fs/debugfs/inode.o fs/debugfs/file.o ; scripts/mod/modpost fs/debugfs/debugfs.o
