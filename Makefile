all:
	#$(MAKE) -C isix_c_examples
	#$(MAKE) -C isix_cpp_examples
	$(MAKE) -C advanced/usbhosthid

clean:
	$(MAKE) -C advanced/usbhosthid clean
	#$(MAKE) -C isix_c_examples clean
	#$(MAKE) -C isix_cpp_examples clean 

program:
	$(MAKE) -C advanced/usbhosthid program

