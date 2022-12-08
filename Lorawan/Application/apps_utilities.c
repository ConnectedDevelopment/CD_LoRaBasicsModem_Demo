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
 * @file      apps_utilities.c
 *
 * @brief     Common Application Helper function implementations
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

#include <stdlib.h>
#include <stdio.h>

#include "apps_utilities.h"
#include "smtc_modem_api_str.h"

#include <logging/log.h>
LOG_MODULE_REGISTER(apps_utilities, CONFIG_LBM_LOG_LEVEL);

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

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC VARIABLES --------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void modem_status_to_string(smtc_modem_status_mask_t modem_status)
{
   LOG_INF("Modem status: ");

   if ((modem_status & SMTC_MODEM_STATUS_BROWNOUT) == SMTC_MODEM_STATUS_BROWNOUT)
   {
      LOG_INF("BROWNOUT ");
   }
   if ((modem_status & SMTC_MODEM_STATUS_CRASH) == SMTC_MODEM_STATUS_CRASH)
   {
      LOG_INF("CRASH ");
   }
   if ((modem_status & SMTC_MODEM_STATUS_MUTE) == SMTC_MODEM_STATUS_MUTE)
   {
      LOG_INF("MUTE ");
   }
   if ((modem_status & SMTC_MODEM_STATUS_JOINED) == SMTC_MODEM_STATUS_JOINED)
   {
      LOG_INF("JOINED ");
   }
   if ((modem_status & SMTC_MODEM_STATUS_SUSPEND) == SMTC_MODEM_STATUS_SUSPEND)
   {
      LOG_INF("SUSPEND ");
   }
   if ((modem_status & SMTC_MODEM_STATUS_UPLOAD) == SMTC_MODEM_STATUS_UPLOAD)
   {
      LOG_INF("UPLOAD ");
   }
   if ((modem_status & SMTC_MODEM_STATUS_JOINING) == SMTC_MODEM_STATUS_JOINING)
   {
      LOG_INF("JOINING ");
   }
   if ((modem_status & SMTC_MODEM_STATUS_STREAM) == SMTC_MODEM_STATUS_STREAM)
   {
      LOG_INF("STREAM ");
   }

   LOG_INF("\r\n\r\n");
}

void modem_class_to_string(smtc_modem_class_t lorawan_class)
{
   switch (lorawan_class)
   {
      case SMTC_MODEM_CLASS_A:
      case SMTC_MODEM_CLASS_B:
      case SMTC_MODEM_CLASS_C:
      {
         LOG_INF("Class: %s\n", smtc_modem_class_to_str(lorawan_class));
         break;
      }

      default:
      {
         LOG_WRN("Unknown Class\n\n");
      }
   }
}

void modem_region_to_string(smtc_modem_region_t region)
{
   switch (region)
   {
      case SMTC_MODEM_REGION_EU_868:
      case SMTC_MODEM_REGION_AS_923_GRP1:
      case SMTC_MODEM_REGION_US_915:
      case SMTC_MODEM_REGION_AU_915:
      case SMTC_MODEM_REGION_CN_470:
      case SMTC_MODEM_REGION_WW2G4:
      case SMTC_MODEM_REGION_AS_923_GRP2:
      case SMTC_MODEM_REGION_AS_923_GRP3:
      case SMTC_MODEM_REGION_IN_865:
      case SMTC_MODEM_REGION_KR_920:
      case SMTC_MODEM_REGION_RU_864:
      case SMTC_MODEM_REGION_CN_470_RP_1_0:
      {
         LOG_INF("Region: %s\n\n", smtc_modem_region_to_str(region));
         break;
      }

      default:
      {
         LOG_WRN("Unknown Region\n\n");
      }
   }
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

