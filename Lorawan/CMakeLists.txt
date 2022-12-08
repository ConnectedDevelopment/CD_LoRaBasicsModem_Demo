# COPYRIGHT 2022 CONNECTED DEVELOPMENT, A DIVISION OF
# EXPONENTIAL TECHNOLOGY GROUP.
#
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

# Define the shield.
set(SHIELD semtech_sx1262_lora)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(Lorawan)

target_sources(app PRIVATE
    Application/main_lorawan.c
    Application/apps_modem_common.c
    Application/apps_modem_event.c
    Application/apps_utilities.c
    Application/smtc_modem_api_str.c
)

#add_subdirectory(../LoraBasicsModem build_lbm)
include(../LoraBasicsModem/CMakeLists.txt)

target_include_directories(app PRIVATE
    ../LoraBasicsModem
    ../LoraBasicsModem/smtc_modem_api
    ../LoraBasicsModem/smtc_modem_core
    ../LoraBasicsModem/smtc_modem_core/device_management
    ../LoraBasicsModem/smtc_modem_core/lorawan_api
    ../LoraBasicsModem/smtc_modem_core/lr1mac
    ../LoraBasicsModem/smtc_modem_core/lr1mac/src
    ../LoraBasicsModem/smtc_modem_core/lr1mac/src/lr1mac_class_b
    ../LoraBasicsModem/smtc_modem_core/lr1mac/src/lr1mac_class_c
    ../LoraBasicsModem/smtc_modem_core/lr1mac/src/services
    ../LoraBasicsModem/smtc_modem_core/lr1mac/src/smtc_real/src
    ../LoraBasicsModem/smtc_modem_core/modem_config
    ../LoraBasicsModem/smtc_modem_core/modem_core
    ../LoraBasicsModem/smtc_modem_core/modem_services
    ../LoraBasicsModem/smtc_modem_core/modem_supervisor
    ../LoraBasicsModem/smtc_modem_core/radio_drivers/sx126x_driver/src
    ../LoraBasicsModem/smtc_modem_core/radio_planner/src
    ../LoraBasicsModem/smtc_modem_core/smtc_modem_crypto
    ../LoraBasicsModem/smtc_modem_core/smtc_modem_crypto/smtc_secure_element
    ../LoraBasicsModem/smtc_modem_core/smtc_modem_crypto/soft_secure_element
    ../LoraBasicsModem/smtc_modem_core/smtc_modem_services
    ../LoraBasicsModem/smtc_modem_core/smtc_modem_services/headers
    ../LoraBasicsModem/smtc_modem_core/smtc_modem_services/src
    ../LoraBasicsModem/smtc_modem_core/smtc_modem_services/src/alc_sync
    ../LoraBasicsModem/smtc_modem_core/smtc_modem_services/src/file_upload
    ../LoraBasicsModem/smtc_modem_core/smtc_modem_services/src/stream
    ../LoraBasicsModem/smtc_modem_core/smtc_ral/src
    ../LoraBasicsModem/smtc_modem_core/smtc_ralf/src
    ../LoraBasicsModem/smtc_modem_hal
    ../RadioDriverHAL
    ../RALBSP
    )

add_compile_definitions(
    ADD_D2D=no
    ADD_MULTICAST=yes
    ADD_SMTC_ALC_SYNC=yes
    ADD_SMTC_FILE_UPLOAD=yes
    ADD_SMTC_STREAM=yes
    CRYPTO=SOFT
    MIDDLEWARE=no
#    MODEM_HAL_DBG_TRACE=0
#    MODEM_HAL_DEEP_DBG_TRACE=0
    RP2_103
    SMTC_D2D
    SMTC_MULTICAST
    SX126X SX1262
    USE_GNSS=no
    USER_DEFINED_JOIN_PARAMETERS
    )

