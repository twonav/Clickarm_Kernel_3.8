cmd_crypto/crypto_hash.o := arm-linux-gnueabihf-ld -EL    -r -o crypto/crypto_hash.o crypto/ahash.o crypto/shash.o ; scripts/mod/modpost crypto/crypto_hash.o
