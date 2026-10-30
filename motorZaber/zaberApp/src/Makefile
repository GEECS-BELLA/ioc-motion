TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

INC += ZaberDriver.h
LIBRARY_IOC = motorZaber
LIB_SRCS += ZaberDriver.cpp
LIB_LIBS += motor asyn
LIB_LIBS += $(EPICS_BASE_IOC_LIBS)
DBD += motorZaberSupport.dbd

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
