bin_PROGRAMS = pomcp

pomcp_SOURCES = \
beliefstate.cpp \
experiment.cpp \
main.cpp \
mcts.cpp \
node.cpp \
simulator.cpp \
utils.cpp \
robotnavigation.cpp

noinst_HEADERS = \
beliefstate.h \
experiment.h \
history.h \
mcts.h \
memorypool.h \
node.h \
simulator.h \
statistic.h \
utils.h \
robotnavigation.h

pomcp_LDFLAGS = $(BOOST_LDFLAGS)

pomcp_LDADD = \
$(BOOST_PROGRAM_OPTIONS_LIB)

pomcp_CPPFLAGS = \
$(BOOST_CPPFLAGS)

DISTCLEANFILES = *~
