all: USNAMachineLearning

clean: clean_USNAMachineLearning

#--------------------------
# TARGET: USNAMachineLearning
#--------------------------
TOP_USNAMachineLearning=./
WD_USNAMachineLearning=$(shell cd ${TOP_USNAMachineLearning};echo `pwd`)
cpp_SRC_USNAMachineLearning+=${WD_USNAMachineLearning}/USNAMachineLearning.cpp
cpp_SRC_USNAMachineLearning+=${WD_USNAMachineLearning}/tinyxml.cpp
cpp_SRC_USNAMachineLearning+=${WD_USNAMachineLearning}/tinystr.cpp 
cpp_SRC_USNAMachineLearning+=${WD_USNAMachineLearning}/tinyxmlerror.cpp
cpp_SRC_USNAMachineLearning+=${WD_USNAMachineLearning}/tinyxmlparser.cpp
OBJS_USNAMachineLearning+=$(cpp_SRC_USNAMachineLearning:.cpp=.cpp.o)
LDFLAGS += -L/usr/lib/i386-linux-gnu
LDFLAGS += -L/usr/lib/i386-linux-gnu/mesa
CFLAGS_USNAMachineLearning+= -iquote./\
 -iquoteSDK/CHeaders/XPLM\
 -iquoteSDK/CHeaders/Widgets\
 -iquote- -iquote/usr/include/\
  -DIBM=0 -DAPL=0 -DLIN=1

DBG=-g

CFLAGS_USNAMachineLearning+=-O6 -x c++ -ansi -m32 

clean_USNAMachineLearning:
	rm -f ${OBJS_USNAMachineLearning}
	rm -f USNAMachineLearning.cpp.d
	rm -f USNAMachineLearning.d
	rm -f tinyxml.cpp.d
	rm -f tinyxml.cpp.o
	rm -f tinystr.cpp.o
	rm -f tinystr.cpp.d
	rm -f tinyxmlerror.cpp.d
	rm -f tinyxmlerror.cpp.o

USNAMachineLearning:
	$(MAKE) -f USNAMachineLearning.mk USNAMachineLearning.xpl TARGET=USNAMachineLearning.xpl\
 CC="g++"  LD="g++"  AR="ar -crs"  SIZE="size" LIBS+="-lGL  -lGLU"

USNAMachineLearning.xpl: ${OBJS_USNAMachineLearning}
	${CC} -m32 -shared ${LDFLAGS} -o USNAMachineLearning.xpl ${OBJS_USNAMachineLearning} ${LIBS}


ifeq (${TARGET}, USNAMachineLearning.xpl)

%.cpp.o: %.cpp
	gcc -c -fPIC ${CFLAGS_USNAMachineLearning} $< -o $@ -MMD
include $(cpp_SRC_USNAMqachineLearning:.cpp=.d)

%.d: %.cpp
	set -e; $(CC) -M $(CFLAGS_USNAMachineLearning) $< \
 | sed 's!\($(*F)\)\.o[ :]*!$(*D)/\1.o $@ : !g' > $@; \
 [ -s $@ ] || rm -f $@

endif
# end Makefile
