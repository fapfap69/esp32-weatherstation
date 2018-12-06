#
# "main" pseudo-component makefile.
#
# (Uses default behaviour of compiling all source files in directory, adding 'include' to include path.)

COMPONENT_EMBED_TXTFILES := ca_cert.pem

ULP_APP_NAME ?= ulp_$(COMPONENT_NAME)

ULP_S_SOURCES = $(addprefix $(COMPONENT_PATH)/ulp/, \
        pulse_cnt.S \
        )
        
ULP_EXP_DEP_OBJECTS := main.o

include $(IDF_PATH)/components/ulp/component_ulp_common.mk