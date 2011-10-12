include Makefile.params

INSTALL_LOCATION=installation

EXTRA_CMAKE_FLAGS=-DCMAKE_INSTALL_PREFIX=`pwd`/../$(INSTALL_LOCATION) -DOMPL_SOURCE:string="$(SOURCE_DIR)"

ompltoplevel:	all installed

include $(shell rospack find mk)/cmake.mk

clean: extra-clean

installed:
	cd build && make install
	touch installed

extra-clean:
	make -f Makefile.ompl clean
	rm -f installed

wipeNoClean:
	make -f Makefile.ompl wipe
	rm -rf $(INSTALL_LOCATION) $(SOURCE_DIR)

wipe: wipeNoClean clean
	-rm -rf bin
