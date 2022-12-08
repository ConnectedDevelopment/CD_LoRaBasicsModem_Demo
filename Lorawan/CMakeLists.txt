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

include(../LoRaBasicsModem_SWL2001/CMakeLists.txt)

target_include_directories(app PRIVATE
    ../LoRaBasicsModem_SWL2001
    ../LoRaBasicsModem_SWL2001/smtc_modem_api
    ../LoRaBasicsModem_SWL2001/smtc_modem_core
    ../LoRaBasicsModem_SWL2001/smtc_modem_core/device_management
    ../LoRaBasicsModem_SWL2001/smtc_modem_core/lorawan_api
    ../LoRaBasicsModem_SWL2001/smtc_modem_core/lr1mac
    ../LoRaBasicsModem_SWL2001/smtc_modem_core/lr1mac/src
    ../LoRaBasicsModem_SWL2001/smtc_modem_core/lr1mac/src/lr1mac_class_b
    ../LoRaBasicsModem_SWL2001/smtc_modem_core/lr1mac/src/lr1mac_class_c
    ../LoRaBasicsModem_SWL2001/smtc_modem_core/lr1mac/src/services
    ../LoRaBasicsModem_SWL2001/smtc_modem_core/lr1mac/src/smtc_real/src
    ../LoRaBasicsModem_SWL2001/smtc_modem_core/modem_config
    ../LoRaBasicsModem_SWL2001/smtc_modem_core/modem_core
    ../LoRaBasicsModem_SWL2001/smtc_modem_core/modem_services
    ../LoRaBasicsModem_SWL2001/smtc_modem_core/modem_supervisor
    ../LoRaBasicsModem_SWL2001/smtc_modem_core/radio_drivers/sx126x_driver/src
    ../LoRaBasicsModem_SWL2001/smtc_modem_core/radio_planner/src
    ../LoRaBasicsModem_SWL2001/smtc_modem_core/smtc_modem_crypto
    ../LoRaBasicsModem_SWL2001/smtc_modem_core/smtc_modem_crypto/smtc_secure_element
    ../LoRaBasicsModem_SWL2001/smtc_modem_core/smtc_modem_crypto/soft_secure_element
    ../LoRaBasicsModem_SWL2001/smtc_modem_core/smtc_modem_services
    ../LoRaBasicsModem_SWL2001/smtc_modem_core/smtc_modem_services/headers
    ../LoRaBasicsModem_SWL2001/smtc_modem_core/smtc_modem_services/src
    ../LoRaBasicsModem_SWL2001/smtc_modem_core/smtc_modem_services/src/alc_sync
    ../LoRaBasicsModem_SWL2001/smtc_modem_core/smtc_modem_services/src/file_upload
    ../LoRaBasicsModem_SWL2001/smtc_modem_core/smtc_modem_services/src/stream
    ../LoRaBasicsModem_SWL2001/smtc_modem_core/smtc_ral/src
    ../LoRaBasicsModem_SWL2001/smtc_modem_core/smtc_ralf/src
    ../LoRaBasicsModem_SWL2001/smtc_modem_hal
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
    RP2_103
    SMTC_D2D
    SMTC_MULTICAST
    SX126X SX1262
    USE_GNSS=no
    USER_DEFINED_JOIN_PARAMETERS
    )

