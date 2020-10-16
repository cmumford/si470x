
CLANG_FORMAT=clang-format

SOURCE_FILES = \
		include/*.h \
		mjs_fs/api_si470x.js \
		src/*.[ch]

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
