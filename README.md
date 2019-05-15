# openocd-0.9.0_maxim_update

# To Compile:
./configure --enable-ftdi
make

# To Connect (from the src folder):
sudo ./openocd -f ../tcl/interface/ftdi/olimex-arm-usb-tiny-h.cfg -f ../tcl/interface/ftdi/olimex-arm-jtag-swd.cfg  -f ../tcl/target/me11.cfg

OR

# To Erase (from the src folder):
sudo ./openocd -f ../tcl/interface/ftdi/olimex-arm-usb-tiny-h.cfg -f ../tcl/interface/ftdi/olimex-arm-jtag-swd.cfg  -f ../tcl/target/me11_erase.cfg

