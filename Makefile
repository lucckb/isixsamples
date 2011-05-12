all:
	$(MAKE) -C isix_c_examples
	$(MAKE) -C isix_cpp_examples

clean:
	$(MAKE) -C isix_c_examples $@
	$(MAKE) -C isix_cpp_examples $@ 

