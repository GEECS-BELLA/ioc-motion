TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

INC += VelmexDriver.h
LIBRARY_IOC = motorVelmex
LIB_SRCS += VelmexDriver.cpp
LIB_LIBS += motor asyn
LIB_LIBS += $(EPICS_BASE_IOC_LIBS)
DBD += motorVelmexSupport.dbd

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
