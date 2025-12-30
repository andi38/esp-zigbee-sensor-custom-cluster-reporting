# https://github.com/espressif/esp-zigbee-sdk/issues/577
idf_component_register(
#  SRC_DIRS  "." "$ENV{IDF_PATH}/examples/zigbee/common/zcl_utility/src"
#  INCLUDE_DIRS "." "$ENV{IDF_PATH}/examples/zigbee/common/zcl_utility/include"
  SRC_DIRS  "." "~/esp/esp-zigbee-sdk/examples/common/zcl_utility/src"
  INCLUDE_DIRS "." "~/esp/esp-zigbee-sdk/examples/common/zcl_utility/include"
)
