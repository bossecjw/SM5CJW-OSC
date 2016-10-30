#/bin/sh
sudo avrdude -c usbtiny -p t85 -U flash:w:Si5351_styr.hex:i
