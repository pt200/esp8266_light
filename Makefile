#
# This is a project Makefile. It is assumed the directory this Makefile resides in is a
# project subdirectory.
#

PROJECT_NAME := light

MY_PROJECT_PATH := $(abspath $(dir $(firstword $(MAKEFILE_LIST))))

EXTRA_COMPONENT_DIRS = $(MY_PROJECT_PATH)/../esp8266_base_code

include $(IDF_PATH)/make/project.mk

