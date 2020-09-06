
CLANG_FORMAT=clang-format

SOURCE_FILES = \
		include/mgos_si470x.h \
		include/port.h \
		include/si470x.h \
		include/si470x_common.h \
		mjs_fs/api_si470x.js \
		src/mgos_si470x.c \
		src/si470x_misc.c \
		src/si470x_misc.h \
		src/si470x.c

.PHONY: format
format:
	${CLANG_FORMAT} -i ${SOURCE_FILES}

docs: doxygen.conf Makefile
	doxygen doxygen.conf

.PHONY: clean
clean:
	rm -rf build docs

.PHONY: tags
tags:
	ctags --extra=+f --languages=+C,+C++ --recurse=yes --links=no
