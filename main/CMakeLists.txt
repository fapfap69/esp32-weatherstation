set(COMPONENT_SRCS "main.cpp")
set(COMPONENT_ADD_INCLUDEDIRS "")
set(COMPONENT_REQUIRES soc nvs_flash ulp)

register_component()

set(ULP_APP_NAME ulp_${COMPONENT_NAME})
set(ULP_S_SOURCES "ulp/pulse_cnt.S")
set(ULP_EXP_DEP_SRCS "main.cpp")
include(${IDF_PATH}/components/ulp/component_ulp_common.cmake)
