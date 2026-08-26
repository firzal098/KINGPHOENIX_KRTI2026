# Webots C++ Controller Makefile for ardupilot_vehicle_controller

CXX_SOURCES = ardupilot_vehicle_controller.cpp webots_vehicle.cpp toojpeg.cpp
INCLUDE = -I"./"

### Auto-detect WEBOTS_HOME if not set
ifeq ($(WEBOTS_HOME),)
  ifneq ($(wildcard /mnt/c/Program\ Files/Webots),)
    WEBOTS_HOME = /mnt/c/Program Files/Webots
  else ifneq ($(wildcard /usr/local/webots),)
    WEBOTS_HOME = /usr/local/webots
  else ifneq ($(wildcard /snap/webots/current/usr/share/webots),)
    WEBOTS_HOME = /snap/webots/current/usr/share/webots
  else ifneq ($(wildcard /opt/webots),)
    WEBOTS_HOME = /opt/webots
  endif
endif

### Do not modify: this includes Webots global Makefile.include
space :=
space +=
WEBOTS_HOME_PATH = $(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))
include $(WEBOTS_HOME_PATH)/resources/Makefile.include
