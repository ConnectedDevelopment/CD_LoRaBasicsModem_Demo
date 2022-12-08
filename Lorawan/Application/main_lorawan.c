/*********************************************************************
* COPYRIGHT 2022 CONNECTED DEVELOPMENT, A DIVISION OF EXPONENTIAL
* TECHNOLOGY GROUP.
*
* SPDX-License-Identifier: Apache-2.0
*
*******************************************************************************
* @file
* @brief  Connected Development implementation of a LoRaWAN Class A demo application.
*
* @details  Copied from the example in the Semtech LoRa Basics Modem SDK at:
*           SWSD001\apps\examples\lorawan\main_lorawan.c
******************************************************************************/

/*!
 * @file      main_lorawan.c
 *
 * @brief     LoRa Basics Modem Class A/C device implementation
 *
 * @copyright
 * The Clear BSD License
 * Copyright Semtech Corporation 2021. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Semtech corporation nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT
 * NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL SEMTECH CORPORATION BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include "main_lorawan.h"
#include "apps_modem_common.h"
#include "apps_modem_event.h"
#include "smtc_board_ralf.h"
#include "apps_utilities.h"
#include "smtc_modem_utilities.h"
#include "smtc_modem_api_str.h"

#include "sx126x_hal_context.h"
#include "modem_context.h"

#include <kernel.h>
#include <irq.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(Main, CONFIG_LBM_LOG_LEVEL);

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*!
 * @brief Stringify constants
 */
#define xstr( a ) str( a )
#define str( a ) #a

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*!
 * @brief Stack identifier
 */
static uint8_t stack_id = 0;

/*!
 * @brief User application data
 */
static uint8_t app_data_buffer[LORAWAN_APP_DATA_MAX_SIZE];

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*!
 * @brief   Send an application frame on LoRaWAN port defined by LORAWAN_APP_PORT
 *
 * @param [in] buffer     Buffer containing the LoRaWAN buffer
 * @param [in] length     Payload length
 * @param [in] confirmed  Send a confirmed or unconfirmed uplink [false : unconfirmed / true : confirmed]
 */
static void send_frame(const uint8_t *buffer, const uint8_t length, const bool confirmed);

/*!
 * @brief Reset event callback
 *
 * @param [in] reset_count reset counter from the modem
 */
static void on_modem_reset(uint16_t reset_count);

/*!
 * @brief Network Joined event callback
 */
static void on_modem_network_joined(void);

/*!
 * @brief Alarm event callback
 */
static void on_modem_alarm(void);

/*!
 * @brief Tx done event callback
 *
 * @param [in] status tx done status @ref smtc_modem_event_txdone_status_t
 */
static void on_modem_tx_done(smtc_modem_event_txdone_status_t status);

/*!
 * @brief Downlink data event callback.
 *
 * @param [in] rssi       RSSI in signed value in dBm + 64
 * @param [in] snr        SNR signed value in 0.25 dB steps
 * @param [in] rx_window  RX window
 * @param [in] port       LoRaWAN port
 * @param [in] payload    Received buffer pointer
 * @param [in] size       Received buffer size
 */
static void on_modem_down_data(int8_t rssi, int8_t snr, smtc_modem_event_downdata_window_t rx_window, uint8_t port,
                               const uint8_t *payload, uint8_t size);

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

/**
 * @brief Main application entry point.
 */
int main(void)
{
   static apps_modem_event_callback_t smtc_event_callback = {
      .adr_mobile_to_static  = NULL,
      .alarm                 = on_modem_alarm,
      .almanac_update        = NULL,
      .down_data             = on_modem_down_data,
      .join_fail             = NULL,
      .joined                = on_modem_network_joined,
      .link_status           = NULL,
      .mute                  = NULL,
      .new_link_adr          = NULL,
      .reset                 = on_modem_reset,
      .set_conf              = NULL,
      .stream_done           = NULL,
      .time_updated_alc_sync = NULL,
      .tx_done               = on_modem_tx_done,
      .upload_done           = NULL,
   };

   unsigned int key;

   /* Initialise the ralf_t object corresponding to the board */
   ralf_t *modem_radio = smtc_board_initialise_and_get_ralf();

   /* Disable IRQ to avoid unwanted behaviour during init */
   key = irq_lock();

   /* Init board and peripherals */
//   hal_mcu_init();
//   smtc_board_init_periph();

   /* Init the Lora Basics Modem event callbacks */
   apps_modem_event_init(&smtc_event_callback);

   /* Init the modem and use apps_modem_event_process as event callback. Please note that the callback
    * will be called immediately after the first call to modem_run_engine because of the reset detection. */
   smtc_modem_init(modem_radio, &apps_modem_event_process);

   /* Re-enable IRQ */
   irq_unlock(key);

   LOG_INF("###### ===== LoRa Basics Modem LoRaWAN Class A/C demo application ==== ######");
   LOG_INF("Version 1.0  Build: %s %s", __DATE__, __TIME__);
   apps_modem_common_display_version_information();

   while (1)
   {
      /* Execute modem runtime, this function must be called again in sleep_time_ms milliseconds or sooner. */
      uint32_t sleep_time_ms = smtc_modem_run_engine();

      /* go in low power */
      k_sleep(K_MSEC(sleep_time_ms));
   }
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static void on_modem_reset(uint16_t reset_count)
{
   LOG_INF("Application parameters:");
   LOG_INF("  - LoRaWAN uplink Fport = %d", LORAWAN_APP_PORT);
   LOG_INF("  - DM report interval   = %d", APP_TX_DUTYCYCLE);
   LOG_INF("  - Confirmed uplink     = %s", (LORAWAN_CONFIRMED_MSG_ON == true) ? "Yes" : "No");

   apps_modem_common_configure_lorawan_params(stack_id);

   ASSERT_SMTC_MODEM_RC(smtc_modem_join_network(stack_id));
}

static void on_modem_network_joined(void)
{
   ASSERT_SMTC_MODEM_RC(smtc_modem_alarm_start_timer(APP_TX_DUTYCYCLE));

   ASSERT_SMTC_MODEM_RC(smtc_modem_adr_set_profile(stack_id, LORAWAN_DEFAULT_DATARATE, adr_custom_list));
}

static void on_modem_alarm(void)
{
   smtc_modem_status_mask_t modem_status;
   uint32_t                 charge        = 0;
   uint8_t                  app_data_size = 0;

   /* Schedule next packet transmission */
   ASSERT_SMTC_MODEM_RC(smtc_modem_alarm_start_timer(APP_TX_DUTYCYCLE));
   LOG_INF("smtc_modem_alarm_start_timer: %d s", APP_TX_DUTYCYCLE);

   ASSERT_SMTC_MODEM_RC(smtc_modem_get_status(stack_id, &modem_status));
   modem_status_to_string(modem_status);

   ASSERT_SMTC_MODEM_RC(smtc_modem_get_charge(&charge));

   app_data_buffer[app_data_size++] = (uint8_t) (charge);
   app_data_buffer[app_data_size++] = (uint8_t) (charge >> 8);
   app_data_buffer[app_data_size++] = (uint8_t) (charge >> 16);
   app_data_buffer[app_data_size++] = (uint8_t) (charge >> 24);

   send_frame(app_data_buffer, app_data_size, LORAWAN_CONFIRMED_MSG_ON);
}

static void on_modem_tx_done(smtc_modem_event_txdone_status_t status)
{
   static uint32_t uplink_count = 0;

   if (status == SMTC_MODEM_EVENT_TXDONE_NOT_SENT)
   {
      LOG_WRN("Uplink was not sent");
   }
   else
   {
      LOG_INF("Uplink count: %d", uplink_count);
      ++uplink_count;
   }
}

static void on_modem_down_data(int8_t rssi, int8_t snr, smtc_modem_event_downdata_window_t rx_window, uint8_t port,
                               const uint8_t *payload, uint8_t size)
{
   LOG_INF("Downlink received:");
   LOG_INF("  - LoRaWAN Fport = %d", port);
   LOG_INF("  - Payload size  = %d", size);
   LOG_INF("  - RSSI          = %d dBm", rssi - 64);
   LOG_INF("  - SNR           = %d dB", snr >> 2);

   switch (rx_window)
   {
      case SMTC_MODEM_EVENT_DOWNDATA_WINDOW_RX1:
      case SMTC_MODEM_EVENT_DOWNDATA_WINDOW_RX2:
      case SMTC_MODEM_EVENT_DOWNDATA_WINDOW_RXC:
      {
         LOG_INF("  - Rx window     = %s", smtc_modem_event_downdata_window_to_str(rx_window));
         break;
      }

      default:
      {
         LOG_ERR("Unexpected event down data window %s",
                 smtc_modem_event_downdata_window_to_str(rx_window));
         break;
      }
   }

   if (size != 0)
   {
      LOG_HEXDUMP_DBG(payload, size, "Payload");
   }
}

static void send_frame(const uint8_t *buffer, const uint8_t length, bool tx_confirmed)
{
   uint8_t tx_max_payload;
   int32_t duty_cycle;

   /* Check if duty cycle is available */
   ASSERT_SMTC_MODEM_RC(smtc_modem_get_duty_cycle_status(&duty_cycle));
   if (duty_cycle < 0)
   {
      LOG_WRN("Duty-cycle limitation - next possible uplink in %d ms", duty_cycle);
      return;
   }

   ASSERT_SMTC_MODEM_RC(smtc_modem_get_next_tx_max_payload(stack_id, &tx_max_payload));
   if (length > tx_max_payload)
   {
      LOG_WRN("Not enough space in buffer - send empty uplink to flush MAC commands");
      ASSERT_SMTC_MODEM_RC(smtc_modem_request_empty_uplink(stack_id, true, LORAWAN_APP_PORT, tx_confirmed));
   }
   else
   {
      LOG_INF("Request uplink");
      ASSERT_SMTC_MODEM_RC(smtc_modem_request_uplink(stack_id, LORAWAN_APP_PORT, tx_confirmed, buffer, length));
   }
}

