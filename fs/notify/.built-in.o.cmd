cmd_fs/notify/built-in.o :=  arm-linux-gnueabihf-ld -EL    -r -o fs/notify/built-in.o fs/notify/fsnotify.o fs/notify/notification.o fs/notify/group.o fs/notify/inode_mark.o fs/notify/mark.o fs/notify/vfsmount_mark.o fs/notify/fdinfo.o fs/notify/dnotify/built-in.o fs/notify/inotify/built-in.o fs/notify/fanotify/built-in.o ; scripts/mod/modpost fs/notify/built-in.o
