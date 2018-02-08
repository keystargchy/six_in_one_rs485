
CC:=arm-none-linux-gnueabi-gcc

all:
	$(CC) -o six_in_one six_in_one.c 

clean:
	rm six_in_one
cp:
	cp six_in_one /home/keystar/samba_share/tftpboot
