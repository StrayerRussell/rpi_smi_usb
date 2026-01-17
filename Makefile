all: smi_usb rpi_smi_adc_test packet_test

smi_usb: smi_usb.c
	gcc -Wall -fcommon -o smi_usb smi_usb.c rpi_dma_utils.c

rpi_smi_adc_test: rpi_smi_adc_test.c
	gcc -Wall -fcommon -o rpi_smi_adc_test rpi_smi_adc_test.c rpi_dma_utils.c

packet_test: packet_test.c
	gcc -o packet_test packet_test.c

clean:
	rm -f smi_usb rpi_smi_adc_test
