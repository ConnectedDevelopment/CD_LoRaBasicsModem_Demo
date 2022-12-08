/*********************************************************************
* COPYRIGHT 2022 CONNECTED DEVELOPMENT, A DIVISION OF EXPONENTIAL
* TECHNOLOGY GROUP.
*
* SPDX-License-Identifier: Apache-2.0
*
*******************************************************************************
* @file
* @brief  Connected Development implementation of the radio HAL functions for
*         the Connected Development SX1262 shield.
*
* @details  Copied from the example in the Semtech LoRa Basics Modem SDK at:
*             SWSD001\shields\SX126X\radio_drivers_hal
******************************************************************************/

/**
 * @file      sx126x_hal.c
 *
 * @brief     Implements the SX126x radio HAL functions
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2022. All rights reserved.
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

#include <stdint.h>
#include <stdbool.h>
#include <zephyr/drivers/spi.h>

#include "sx126x_hal.h"
#include "sx126x_hal_context.h"

#include <logging/log.h>
LOG_MODULE_REGISTER(RadioHAL, CONFIG_LBM_LOG_LEVEL);

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/**
 * Commands Interface
 */
typedef enum sx126x_commands_e
{
    // Operational Modes Functions
    SX126X_SET_SLEEP                  = 0x84,
    SX126X_SET_STANDBY                = 0x80,
    SX126X_SET_FS                     = 0xC1,
    SX126X_SET_TX                     = 0x83,
    SX126X_SET_RX                     = 0x82,
    SX126X_SET_STOP_TIMER_ON_PREAMBLE = 0x9F,
    SX126X_SET_RX_DUTY_CYCLE          = 0x94,
    SX126X_SET_CAD                    = 0xC5,
    SX126X_SET_TX_CONTINUOUS_WAVE     = 0xD1,
    SX126X_SET_TX_INFINITE_PREAMBLE   = 0xD2,
    SX126X_SET_REGULATOR_MODE         = 0x96,
    SX126X_CALIBRATE                  = 0x89,
    SX126X_CALIBRATE_IMAGE            = 0x98,
    SX126X_SET_PA_CFG                 = 0x95,
    SX126X_SET_RX_TX_FALLBACK_MODE    = 0x93,
    // Registers and buffer Access
    SX126X_WRITE_REGISTER = 0x0D,
    SX126X_READ_REGISTER  = 0x1D,
    SX126X_WRITE_BUFFER   = 0x0E,
    SX126X_READ_BUFFER    = 0x1E,
    // DIO and IRQ Control Functions
    SX126X_SET_DIO_IRQ_PARAMS         = 0x08,
    SX126X_GET_IRQ_STATUS             = 0x12,
    SX126X_CLR_IRQ_STATUS             = 0x02,
    SX126X_SET_DIO2_AS_RF_SWITCH_CTRL = 0x9D,
    SX126X_SET_DIO3_AS_TCXO_CTRL      = 0x97,
    // RF Modulation and Packet-Related Functions
    SX126X_SET_RF_FREQUENCY          = 0x86,
    SX126X_SET_PKT_TYPE              = 0x8A,
    SX126X_GET_PKT_TYPE              = 0x11,
    SX126X_SET_TX_PARAMS             = 0x8E,
    SX126X_SET_MODULATION_PARAMS     = 0x8B,
    SX126X_SET_PKT_PARAMS            = 0x8C,
    SX126X_SET_CAD_PARAMS            = 0x88,
    SX126X_SET_BUFFER_BASE_ADDRESS   = 0x8F,
    SX126X_SET_LORA_SYMB_NUM_TIMEOUT = 0xA0,
    // Communication Status Information
    SX126X_GET_STATUS           = 0xC0,
    SX126X_GET_RX_BUFFER_STATUS = 0x13,
    SX126X_GET_PKT_STATUS       = 0x14,
    SX126X_GET_RSSI_INST        = 0x15,
    SX126X_GET_STATS            = 0x10,
    SX126X_RESET_STATS          = 0x00,
    // Miscellaneous
    SX126X_GET_DEVICE_ERRORS = 0x17,
    SX126X_CLR_DEVICE_ERRORS = 0x07,
} sx126x_commands_t;


/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

// SX126X_SET_SLEEP command.
#define OPCODE_SET_SLEEP      0x84
#define CMD_SIZE_SET_SLEEP    2

// SX126X_READ_REGISTER command opcode and status size.
#define OPCODE_READ_REGISTER        0x1D
#define STATUS_SIZE_READ_REGISTER   4

// SX126X_READ_BUFFER command opcode and status size.
#define OPCODE_READ_BUFFER          0x1E
#define STATUS_SIZE_READ_BUFFER     3

// Status size of other read/get commands.
#define STATUS_SIZE_READ_CMD        2


/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

typedef enum
{
   RADIO_SLEEP,
   RADIO_AWAKE
} radio_mode_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

static volatile radio_mode_t radio_mode = RADIO_AWAKE;
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

static void Sx126xHalWaitOnBusy(const struct gpio_dt_spec *gpioBusy);
static void Sx126xHalCheckDeviceReady(const sx126x_hal_context_t *sx126xContext);
char *Sx126xCmdName(sx126x_commands_t cmd);

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/**
 * Radio data transfer - Write command and data to the radio.
 *
 * @param [in] context          Radio implementation parameters.
 * @param [in] command          Pointer to the write command to be transmitted.
 * @param [in] command_length   Command buffer size to be transmitted.
 * @param [in] data             Pointer to the data buffer to be transmitted.
 * @param [in] data_length      Data buffer size to be transmitted.
 *
 * @returns Operation status
 */
sx126x_hal_status_t sx126x_hal_write(const void *context, const uint8_t *command, const uint16_t command_length,
                                     const uint8_t *data, const uint16_t data_length)

{
   const sx126x_hal_context_t *sx126xContext = (const sx126x_hal_context_t *) context;
   int ret;

   struct spi_buf      txBuf[2];
   struct spi_buf_set  txBuffers;

   txBuf[0].buf = (void *) command;
   txBuf[0].len = command_length;
   txBuf[1].buf = (void *) data;
   txBuf[1].len = data_length;
   txBuffers.buffers = txBuf;
   txBuffers.count = 2;

   // Wait for readiness.
   Sx126xHalCheckDeviceReady(sx126xContext);

   LOG_HEXDUMP_DBG(txBuffers.buffers[0].buf, txBuffers.buffers[0].len, Sx126xCmdName(command[0]));
   if (txBuffers.buffers[1].buf != 0)
   {
      LOG_HEXDUMP_DBG(txBuffers.buffers[1].buf, txBuffers.buffers[1].len, "Write data:");
   }

   // Write the command and data to the SX126X.
   ret = spi_transceive_dt(&sx126xContext->spiSpec, &txBuffers, NULL);

   if (ret < 0)
   {
      LOG_ERR("SPI write error: %d", ret);
      return SX126X_HAL_STATUS_ERROR;
   }

   // Check whether the command is a sleep command to keep the state up to date.
   if ((command[0] == OPCODE_SET_SLEEP) && (command_length == CMD_SIZE_SET_SLEEP))
   {
      radio_mode = RADIO_SLEEP;
   }

   return SX126X_HAL_STATUS_OK;
}

/**
 * Radio data transfer - Read data from radio the radio.
 *
 * @param [in] context          Radio implementation parameters.
 * @param [in] command          Pointer to the read command to be transmitted.
 * @param [in] command_length   Command buffer size to be transmitted.
 * @param [in] data             Pointer to the data buffer to be received.
 * @param [in] data_length      Data buffer size to be received.
 *
 * @returns Operation status
 */
sx126x_hal_status_t sx126x_hal_read(const void *context, const uint8_t *command, const uint16_t command_length,
                                    uint8_t *data, const uint16_t data_length)
{
   const sx126x_hal_context_t *sx126xContext = (const sx126x_hal_context_t *) context;
   int ret;

   struct spi_buf      txBuf[2];
   struct spi_buf_set  txBuffers;
   struct spi_buf      rxBuf[2];
   struct spi_buf_set  rxBuffers;
   uint8_t rxStatus[4];    // Buffer to receive the status

   txBuf[0].buf = (void *) command;
   txBuf[0].len = command_length;
   txBuf[1].buf = NULL;
   txBuf[1].len = 0;
   txBuffers.buffers = txBuf;
   txBuffers.count = 2;

   if (command[0] == OPCODE_READ_REGISTER)
   {
      rxBuf[0].buf = rxStatus;
      rxBuf[0].len = STATUS_SIZE_READ_REGISTER;
      LOG_HEXDUMP_DBG(txBuffers.buffers[0].buf, txBuffers.buffers[0].len, "READ_REGISTER:");
   }
   else if (command[0] == OPCODE_READ_BUFFER)
   {
      rxBuf[0].buf = rxStatus;
      rxBuf[0].len = STATUS_SIZE_READ_BUFFER;
      LOG_HEXDUMP_DBG(txBuffers.buffers[0].buf, txBuffers.buffers[0].len, "READ_BUFFER:");
   }
   else  // Other read command.
   {
      rxBuf[0].buf = rxStatus;
      rxBuf[0].len = STATUS_SIZE_READ_CMD;
      LOG_HEXDUMP_DBG(txBuffers.buffers[0].buf, txBuffers.buffers[0].len, Sx126xCmdName(command[0]));
   }

   rxBuf[1].buf = (void *) data;
   rxBuf[1].len = data_length;
   rxBuffers.buffers = rxBuf;
   rxBuffers.count = 2;

   // Wait for readiness.
   Sx126xHalCheckDeviceReady(sx126xContext);

   // Write the command and read the data from the SX126X.
   ret = spi_transceive_dt(&sx126xContext->spiSpec, &txBuffers, &rxBuffers);

   if (ret < 0)
   {
      LOG_ERR("SPI read error: %d", ret);
      return SX126X_HAL_STATUS_ERROR;
   }

   LOG_HEXDUMP_DBG(rxBuf[0].buf, rxBuf[0].len, "Read status:");
   LOG_HEXDUMP_DBG(rxBuffers.buffers[1].buf, rxBuffers.buffers[1].len, "Read data:");

   return SX126X_HAL_STATUS_OK;
}

/**
 * Reset the radio.
 *
 * @param [in] context Radio implementation parameters.
 *
 * @returns Operation status
 */
sx126x_hal_status_t sx126x_hal_reset(const void *context)
{
   const sx126x_hal_context_t *sx126xContext = (const sx126x_hal_context_t *) context;

   LOG_DBG("Reset sx126x on port %s pin %u", sx126xContext->gpioReset.port->name, sx126xContext->gpioReset.pin);

   gpio_pin_set_dt(&sx126xContext->gpioReset, 1);
   k_sleep(K_USEC(2000));
   gpio_pin_set_dt(&sx126xContext->gpioReset, 0);

   // Reset wakes up radio
   radio_mode = RADIO_AWAKE;

   return SX126X_HAL_STATUS_OK;
}

/**
 * Wake the radio up.
 *
 * @param [in] context Radio implementation parameters.
 *
 * @returns Operation status
 */
sx126x_hal_status_t sx126x_hal_wakeup(const void *context)
{
   const sx126x_hal_context_t *sx126xContext = (const sx126x_hal_context_t *) context;

   Sx126xHalCheckDeviceReady(sx126xContext);
   return SX126X_HAL_STATUS_OK;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/**
 * @brief Wait until radio busy pin returns to 0
 */
static void Sx126xHalWaitOnBusy(const struct gpio_dt_spec *gpioBusy)
{
   while (gpio_pin_get_dt(gpioBusy) == 1)
   {
      k_sleep(K_USEC(1000));
   };
}

static void Sx126xHalCheckDeviceReady(const sx126x_hal_context_t *sx126xContext)
{
   if (radio_mode != RADIO_SLEEP)
   {
      Sx126xHalWaitOnBusy(&sx126xContext->gpioBusy);
   }
   else
   {
      LOG_DBG("Wake up sx126x on port %s pin %u", sx126xContext->gpioCs.port->name,
              sx126xContext->gpioCs.pin);

      // Busy is HIGH in sleep mode, wake-up the device with a small glitch on NSS.
      gpio_pin_set_dt(&sx126xContext->gpioCs, 1);
      Sx126xHalWaitOnBusy(&sx126xContext->gpioBusy);
      gpio_pin_set_dt(&sx126xContext->gpioCs, 0);
      radio_mode = RADIO_AWAKE;

      k_sleep(K_USEC(1000));
   }
}

char *Sx126xCmdName(sx126x_commands_t cmd)
{
   switch (cmd)
   {
      case SX126X_SET_SLEEP:                    return "SET_SLEEP";
      case SX126X_SET_STANDBY:                  return "SET_STANDBY";
      case SX126X_SET_FS:                       return "SET_FS";
      case SX126X_SET_TX:                       return "SET_TX";
      case SX126X_SET_RX:                       return "SET_RX";
      case SX126X_SET_STOP_TIMER_ON_PREAMBLE:   return "SET_STOP_TIMER_ON_PREAMBLE";
      case SX126X_SET_RX_DUTY_CYCLE:            return "SET_RX_DUTY_CYCLE";
      case SX126X_SET_CAD:                      return "SET_CAD";
      case SX126X_SET_TX_CONTINUOUS_WAVE:       return "SET_TX_CONTINUOUS_WAVE";
      case SX126X_SET_TX_INFINITE_PREAMBLE:     return "SET_TX_INFINITE_PREAMBLE";
      case SX126X_SET_REGULATOR_MODE:           return "SET_REGULATOR_MODE";
      case SX126X_CALIBRATE:                    return "CALIBRATE";
      case SX126X_CALIBRATE_IMAGE:              return "CALIBRATE_IMAGE";
      case SX126X_SET_PA_CFG:                   return "SET_PA_CFG";
      case SX126X_SET_RX_TX_FALLBACK_MODE:      return "SET_RX_TX_FALLBACK_MODE";
      case SX126X_WRITE_REGISTER:               return "WRITE_REGISTER";
      case SX126X_READ_REGISTER:                return "READ_REGISTER";
      case SX126X_WRITE_BUFFER:                 return "WRITE_BUFFER";
      case SX126X_READ_BUFFER:                  return "READ_BUFFER";
      case SX126X_SET_DIO_IRQ_PARAMS:           return "SET_DIO_IRQ_PARAMS";
      case SX126X_GET_IRQ_STATUS:               return "GET_IRQ_STATUS";
      case SX126X_CLR_IRQ_STATUS:               return "CLR_IRQ_STATUS";
      case SX126X_SET_DIO2_AS_RF_SWITCH_CTRL:   return "SET_DIO2_AS_RF_SWITCH_CTRL";
      case SX126X_SET_DIO3_AS_TCXO_CTRL:        return "SET_DIO3_AS_TCXO_CTRL";
      case SX126X_SET_RF_FREQUENCY:             return "SET_RF_FREQUENCY";
      case SX126X_SET_PKT_TYPE:                 return "SET_PKT_TYPE";
      case SX126X_GET_PKT_TYPE:                 return "GET_PKT_TYPE";
      case SX126X_SET_TX_PARAMS:                return "SET_TX_PARAMS";
      case SX126X_SET_MODULATION_PARAMS:        return "SET_MODULATION_PARAMS";
      case SX126X_SET_PKT_PARAMS:               return "SET_PKT_PARAMS";
      case SX126X_SET_CAD_PARAMS:               return "SET_CAD_PARAMS";
      case SX126X_SET_BUFFER_BASE_ADDRESS:      return "SET_BUFFER_BASE_ADDRESS";
      case SX126X_SET_LORA_SYMB_NUM_TIMEOUT:    return "SET_LORA_SYMB_NUM_TIMEOUT";
      case SX126X_GET_STATUS:                   return "GET_STATUS";
      case SX126X_GET_RX_BUFFER_STATUS:         return "GET_RX_BUFFER_STATUS";
      case SX126X_GET_PKT_STATUS:               return "GET_PKT_STATUS";
      case SX126X_GET_RSSI_INST:                return "GET_RSSI_INST";
      case SX126X_GET_STATS:                    return "GET_STATS";
      case SX126X_RESET_STATS:                  return "RESET_STATS";
      case SX126X_GET_DEVICE_ERRORS:            return "GET_DEVICE_ERRORS";
      case SX126X_CLR_DEVICE_ERRORS:            return "CLR_DEVICE_ERRORS";
   }

   return "UNKNOWN CMD";
}

