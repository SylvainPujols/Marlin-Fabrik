TARGET=scheme
ROOTDIR=sfsn
DIRNAME=`basename $(PWD)`
CC=`which gcc`
LD=`which gcc`
RM=`which rm` -f

SRCDIR=src
INCDIR=include
TESTDIR=testing

GARBAGE=*~ $(SRCDIR)/*~ $(INCDIR)/*~ $(TESTDIR)/*~

INCLUDE=-I$(INCDIR)

# Pour activer les sorties INFO_MSG, ajouter -DVERBOSE aux CFLAGS 
CFLAGS=-Wall -ansi $(INCLUDE)
LFLAGS=-lm

CFLAGS_DBG=$(CFLAGS) -g -DDEBUG -Wall
CFLAGS_RLS=$(CFLAGS)

SRC=$(wildcard $(SRCDIR)/*.c)

OBJ_DBG=$(SRC:.c=.dbg)
OBJ_RLS=$(SRC:.c=.rls)

all : 
	@echo "in " $(DIRNAME)
	@echo ""
	@echo "Usage:"
	@echo ""
	@echo "make debug   => build DEBUG   version"
	@echo "make release => build RELEASE version"
	@echo "make clean   => clean everything"
	@echo "make tarball => produce archive"

debug   : $(OBJ_DBG)
	$(LD) $^ $(LFLAGS) -o $(TARGET)

release : $(OBJ_RLS)
	$(LD) $^ $(LFLAGS) -o $(TARGET)

%.dbg : %.c
	$(CC) $< $(CFLAGS_DBG) -c -o $(basename $<).dbg

%.rls : %.c
	$(CC) $< $(CFLAGS_RLS) -c -o $(basename $<).rls

clean : 
	$(RM) $(TARGET) $(SRCDIR)/*.orig $(SRCDIR)/*.dbg $(SRCDIR)/*.rls $(GARBAGE) scheme-`whoami`-*.tgz

tarball : 
	make clean 
	cd .. && tar -czvf scheme-`whoami`-`date +%d-%m-%H-%M`.tgz $(DIRNAME) && cd $(DIRNAME) && mv ../scheme-`whoami`-*.tgz .
