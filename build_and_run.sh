mkdir qemu/build
cd qemu/build/
../configure --target-list=arm-softmmu
make

cd ../../firmware/code
make
make qemu_start