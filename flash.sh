sudo avrdude -c usbasp -p m328p -U flash:w:grelec_elektronika_2.0.ino.eightanaloginputs.hex -U lfuse:w:0xe2:m -U hfuse:w:0xdf:m -U efuse:w:0xff:m 
