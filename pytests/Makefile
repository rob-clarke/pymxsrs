LIBNAME=pymxsrs

test: ../target/release/lib$(LIBNAME).so
	ln -fs $< $(LIBNAME).so
	python3 test.py

test-debug: ../target/debug/lib$(LIBNAME).so
	ln -fs $< $(LIBNAME).so
	python3 test.py

../target/debug/lib$(LIBNAME).so: ../$(LIBNAME)/src/*
	cd .. && cargo build

../target/release/lib$(LIBNAME).so: ../$(LIBNAME)/src/*
	cd .. && cargo build --release

.PHONY: test test-debug
