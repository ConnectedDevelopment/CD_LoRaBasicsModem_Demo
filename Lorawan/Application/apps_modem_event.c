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
*           SWSD001\apps\common
******************************************************************************/

/*!
 * @file      apps_modem_event.c
 *
 * @brief     LoRa Basics Modem event manager implementation
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

#include "apps_modem_event.h"
#include "smtc_modem_api_str.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(apps_modem_event, CONFIG_LBM_LOG_LEVEL);

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

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
 * @brief Lora Basics Modem event callback functions
 */
apps_modem_event_callback_t *apps_modem_event_callback;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void apps_modem_event_init(apps_modem_event_callback_t *event_callback)
{
   apps_modem_event_callback = event_callback;
}

void apps_modem_event_process(void)
{
   smtc_modem_event_t       current_event;
   smtc_modem_return_code_t return_code = SMTC_MODEM_RC_OK;
   uint8_t                  event_pending_count;

   do
   {
      /* Read modem event */
      return_code = smtc_modem_get_event(&current_event, &event_pending_count);

      if (return_code == SMTC_MODEM_RC_OK)
      {
         if (apps_modem_event_callback != NULL)
         {
            switch (current_event.event_type)
            {
               case SMTC_MODEM_EVENT_RESET:
                  LOG_INF("###### ===== BASICS MODEM RESET EVENT ==== ######");
                  LOG_INF("Reset count : %u \n", current_event.event_data.reset.count);
                  if (apps_modem_event_callback->reset != NULL)
                  {
                     apps_modem_event_callback->reset(current_event.event_data.reset.count);
                  }
                  break;

               case SMTC_MODEM_EVENT_ALARM:
                  LOG_INF("###### ===== ALARM EVENT ==== ######\n");
                  if (apps_modem_event_callback->alarm != NULL)
                  {
                     apps_modem_event_callback->alarm();
                  }
                  break;

               case SMTC_MODEM_EVENT_JOINED:
                  LOG_INF("###### ===== JOINED EVENT ==== ######\n");
                  if (apps_modem_event_callback->joined != NULL)
                  {
                     apps_modem_event_callback->joined();
                  }
                  break;

               case SMTC_MODEM_EVENT_JOINFAIL:
                  LOG_INF("###### ===== JOINED FAIL EVENT ==== ######\n");
                  if (apps_modem_event_callback->join_fail != NULL)
                  {
                     apps_modem_event_callback->join_fail();
                  }
                  break;

               case SMTC_MODEM_EVENT_TXDONE:
                  LOG_INF("###### ===== TX DONE EVENT ==== ######");
                  switch (current_event.event_data.txdone.status)
                  {
                     case SMTC_MODEM_EVENT_TXDONE_NOT_SENT:
                        LOG_ERR("TX Done status: %s\n", smtc_modem_event_txdone_status_to_str(
                                   current_event.event_data.txdone.status));
                        break;

                     case SMTC_MODEM_EVENT_TXDONE_SENT:
                     case SMTC_MODEM_EVENT_TXDONE_CONFIRMED:
                     default:
                        LOG_INF("TX Done status: %s\n", smtc_modem_event_txdone_status_to_str(
                                   current_event.event_data.txdone.status));
                        break;
                  }
                  if (apps_modem_event_callback->tx_done != NULL)
                  {
                     apps_modem_event_callback->tx_done(current_event.event_data.txdone.status);
                  }
                  break;

               case SMTC_MODEM_EVENT_DOWNDATA:
                  LOG_INF("###### ===== DOWNLINK EVENT ==== ######");
                  LOG_INF("Rx window: %s", smtc_modem_event_downdata_window_to_str(
                             current_event.event_data.downdata.window));
                  LOG_INF("Rx port: %d", current_event.event_data.downdata.fport);
                  LOG_INF("Rx RSSI: %d", current_event.event_data.downdata.rssi - 64);
                  LOG_INF("Rx SNR: %d\n", current_event.event_data.downdata.snr / 4);

                  if (apps_modem_event_callback->down_data != NULL)
                  {
                     apps_modem_event_callback->down_data(
                        current_event.event_data.downdata.rssi, current_event.event_data.downdata.snr,
                        current_event.event_data.downdata.window, current_event.event_data.downdata.fport,
                        current_event.event_data.downdata.data, current_event.event_data.downdata.length);
                  }
                  break;

               case SMTC_MODEM_EVENT_UPLOADDONE:
                  LOG_INF("###### ===== UPLOAD DONE EVENT ==== ######");
                  LOG_INF("Upload status: %s\n", smtc_modem_event_uploaddone_status_to_str(
                             current_event.event_data.uploaddone.status));
                  if (apps_modem_event_callback->upload_done != NULL)
                  {
                     apps_modem_event_callback->upload_done(current_event.event_data.uploaddone.status);
                  }
                  break;

               case SMTC_MODEM_EVENT_SETCONF:
                  LOG_INF("###### ===== SET CONF EVENT ==== ######");
                  LOG_INF("Tag: %s\n",
                          smtc_modem_event_setconf_tag_to_str(current_event.event_data.setconf.tag));
                  if (apps_modem_event_callback->set_conf != NULL)
                  {
                     apps_modem_event_callback->set_conf(current_event.event_data.setconf.tag);
                  }
                  break;

               case SMTC_MODEM_EVENT_MUTE:
                  LOG_INF("###### ===== MUTE EVENT ==== ######");
                  LOG_INF("Mute: %s\n",
                          smtc_modem_event_mute_status_to_str(current_event.event_data.mute.status));
                  if (apps_modem_event_callback->mute != NULL)
                  {
                     apps_modem_event_callback->mute(current_event.event_data.mute.status);
                  }
                  break;

               case SMTC_MODEM_EVENT_STREAMDONE:
                  LOG_INF("###### ===== STREAM DONE EVENT ==== ######\n");
                  if (apps_modem_event_callback->stream_done != NULL)
                  {
                     apps_modem_event_callback->stream_done();
                  }
                  break;

               case SMTC_MODEM_EVENT_TIME:
                  LOG_INF("###### ===== TIME EVENT ==== ######");
                  LOG_INF("Time: %s\n",
                          smtc_modem_event_time_status_to_str(current_event.event_data.time.status));
                  if (apps_modem_event_callback->time_updated_alc_sync != NULL)
                  {
                     apps_modem_event_callback->time_updated_alc_sync(current_event.event_data.time.status);
                  }
                  break;

               case SMTC_MODEM_EVENT_TIMEOUT_ADR_CHANGED:
                  LOG_INF("###### ===== ADR CHANGED EVENT ==== ######\n");
                  if (apps_modem_event_callback->adr_mobile_to_static != NULL)
                  {
                     apps_modem_event_callback->adr_mobile_to_static();
                  }
                  break;

               case SMTC_MODEM_EVENT_NEW_LINK_ADR:
                  LOG_INF("###### ===== NEW LINK ADR EVENT ==== ######\n");
                  if (apps_modem_event_callback->new_link_adr != NULL)
                  {
                     apps_modem_event_callback->new_link_adr();
                  }
                  break;

               case SMTC_MODEM_EVENT_LINK_CHECK:
                  LOG_INF("###### ===== LINK CHECK EVENT ==== ######");
                  LOG_INF("Link status: %s", smtc_modem_event_link_check_status_to_str(
                             current_event.event_data.link_check.status));
                  LOG_INF("Margin: %d dB", current_event.event_data.link_check.margin);
                  LOG_INF("Number of gateways: %d\n", current_event.event_data.link_check.gw_cnt);
                  if (apps_modem_event_callback->link_status != NULL)
                  {
                     apps_modem_event_callback->link_status(current_event.event_data.link_check.status,
                                                            current_event.event_data.link_check.margin,
                                                            current_event.event_data.link_check.gw_cnt);
                  }
                  break;

               case SMTC_MODEM_EVENT_ALMANAC_UPDATE:
                  LOG_INF("###### ===== ALMANAC UPDATE EVENT ==== ######");
                  LOG_INF("Almanac update status: %s\n",
                          smtc_modem_event_almanac_update_status_to_str(
                             current_event.event_data.almanac_update.status));
                  if (apps_modem_event_callback->almanac_update != NULL)
                  {
                     apps_modem_event_callback->almanac_update(current_event.event_data.almanac_update.status);
                  }
                  break;

               case SMTC_MODEM_EVENT_USER_RADIO_ACCESS:
                  LOG_INF("###### ===== USER RADIO ACCESS EVENT ==== ######\n");
                  if (apps_modem_event_callback->user_radio_access != NULL)
                  {
                     apps_modem_event_callback->user_radio_access(
                        current_event.event_data.user_radio_access.timestamp_ms,
                        current_event.event_data.user_radio_access.status);
                  }
                  break;

               case SMTC_MODEM_EVENT_CLASS_B_PING_SLOT_INFO:
                  LOG_INF("###### ===== CLASS B PING SLOT INFO EVENT ==== ######");
                  LOG_INF("Class B ping slot status: %s\n",
                          smtc_modem_event_class_b_ping_slot_status_to_str(
                             current_event.event_data.class_b_ping_slot_info.status));
                  if (apps_modem_event_callback->class_b_ping_slot_info != NULL)
                  {
                     apps_modem_event_callback->class_b_ping_slot_info(
                        current_event.event_data.class_b_ping_slot_info.status);
                  }
                  break;

               case SMTC_MODEM_EVENT_CLASS_B_STATUS:
                  LOG_INF("###### ===== CLASS B STATUS EVENT ==== ######");
                  LOG_INF(
                     "Class B status: %s\n",
                     smtc_modem_event_class_b_status_to_str(current_event.event_data.class_b_status.status));
                  if (apps_modem_event_callback->class_b_status != NULL)
                  {
                     apps_modem_event_callback->class_b_status(current_event.event_data.class_b_status.status);
                  }
                  break;

               case SMTC_MODEM_EVENT_MIDDLEWARE_1:
                  LOG_INF("###### ===== MIDDLEWARE_1 EVENT ==== ######\n");
                  if (apps_modem_event_callback->middleware_1 != NULL)
                  {
                     apps_modem_event_callback->middleware_1(
                        current_event.event_data.middleware_event_status.status);
                  }
                  break;

               case SMTC_MODEM_EVENT_MIDDLEWARE_2:
                  LOG_INF("###### ===== MIDDLEWARE_2 EVENT ==== ######\n");
                  if (apps_modem_event_callback->middleware_2 != NULL)
                  {
                     apps_modem_event_callback->middleware_2(
                        current_event.event_data.middleware_event_status.status);
                  }
                  break;

               case SMTC_MODEM_EVENT_MIDDLEWARE_3:
                  LOG_INF("###### ===== MIDDLEWARE_3 EVENT ==== ######\n");
                  if (apps_modem_event_callback->middleware_3 != NULL)
                  {
                     apps_modem_event_callback->middleware_3(
                        current_event.event_data.middleware_event_status.status);
                  }
                  break;

               case SMTC_MODEM_EVENT_NONE:
                  break;

               default:
                  LOG_INF("###### ===== UNKNOWN EVENT %u ==== ######\n", current_event.event_type);
                  break;
            }
         }
         else
         {
            LOG_ERR("lora_basics_modem_event_callback not defined %u", current_event.event_type);
         }
      }
      else
      {
         LOG_ERR("smtc_modem_get_event != SMTC_MODEM_RC_OK");
      }
   } while ((return_code == SMTC_MODEM_RC_OK) && (current_event.event_type != SMTC_MODEM_EVENT_NONE));
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

