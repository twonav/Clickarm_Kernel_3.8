cmd_net/netlink/built-in.o :=  arm-linux-gnueabihf-ld -EL    -r -o net/netlink/built-in.o net/netlink/af_netlink.o net/netlink/genetlink.o ; scripts/mod/modpost net/netlink/built-in.o
