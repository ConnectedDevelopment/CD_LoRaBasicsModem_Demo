# CD_LoRaBasicsModem_Demo
Connected Development sample LoRaWAN demo using Semtech LoRa Basics Modem.

## Description

The application will automatically start a procedure to join a LoRaWAN network (see [configuration](Lorawan/Application/lorawan_key_config.h)).

Once a network is joined (i.e. when the corresponding event is triggered), uplinks are sent periodically. This periodic action is based on the LoRa Basics Modem alarm functionality. Each time the alarm-related event is triggered, the application requests an uplink.

The content of the uplink is the value read out from the charge counter by calling `smtc_modem_get_charge()`.

The application is also capable of displaying data and meta-data of a received downlink.

## Configuration

Several parameters can be updated in `main_lorawan.h` header file:

| Constant                   | Description                                                                   | Possible values  | Default Value |
| -------------------------- | ----------------------------------------------------------------------------- | ---------------- | ------------- |
| `LORAWAN_APP_PORT`         | LoRaWAN FPort used for the uplink messages                                    | [1, 223]         | 2             |
| `LORAWAN_CONFIRMED_MSG_ON` | Request a confirmation from the LNS that the uplink message has been received | {`true`,`false`} | `false`       |
| `APP_TX_DUTYCYCLE`         | Delay in second between two uplinks                                           | `uint32_t`       | 60            |

## LoRaWAN configuration

The LoRaWAN Join configuration is handled by the function `apps_modem_common_configure_lorawan_params()`.

The join parameters (DevEUI, JoinEUI and AppKey) are taken from [lorawan_key_config.h](Lorawan/Application/lorawan_key_config.h).

| Constant             | Description             | Possible values                             | Default value                                                                                      | Note                                              |
| -------------------- | ----------------------- | ------------------------------------------- | -------------------------------------------------------------------------------------------------- | ------------------------------------------------- |
| `LORAWAN_DEVICE_EUI` | LoRaWAN device EUI      | Any 8 bytes c-array                         | `{0xFE, 0xFF, 0xFF, 0xFF, 0xFD, 0xFF, 0x00, 0x00}`                                                 | Used if `USER_DEFINED_JOIN_PARAMETERS` is defined |
| `LORAWAN_JOIN_EUI`   | LoRaWAN join EUI        | Any 8 bytes c-array                         | `{0x00, 0x16, 0xC0, 0x01, 0xFF, 0xFE, 0x00, 0x01}`                                                 | Used if `USER_DEFINED_JOIN_PARAMETERS` is defined |
| `LORAWAN_APP_KEY`    | LoRaWAN application key | Any 16 bytes c-array                        | `{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}` | Used if `USER_DEFINED_JOIN_PARAMETERS` is defined |
| `LORAWAN_REGION`     | LoRaWAN region          | Values in `smtc_modem_region_t` enumeration | `SMTC_MODEM_REGION_EU_868`                                                                         | See supported LoRaWAN regions below               |
| `LORAWAN_CLASS`      | LoRaWAN class           | Values in `smtc_modem_class_t` enumeration  | `SMTC_MODEM_CLASS_A`                                                                               | All LoRaWAN classes (A, B & C) are supported      |

The default build supports the US915 LoRaWAN region.  Other regions can be selected by modifying the build.

## LoRa Basics Modem event management

When LoRa Basics Modem is initialized, a callback is given as parameter to `smtc_modem_init()` so the application can be informed of events. In a final application, it is up to the user to implement this function.

In this demo application, this function is defined in `apps_modem_event_process()` implemented in [apps_modem_event.c](Lorawan/Application/apps_modem_event.c). Each example needs to implement the functions defined in `apps_modem_event_callback_t` that are useful to it - others have to be set to `NULL`.

## Expected Behavior

Here follow the steps that shall be seen in logs to indicate the expected behavior of the application:

### Device starts and reset

 ```
 INFO: Modem Initialization
 INFO: ###### ===== LoRa Basics Modem LoRaWAN Class A/C demo application ==== ######
 INFO: ###### ===== BASICS MODEM RESET EVENT ==== ######
 ```

Following this print you shall find application and LoRaWAN parameters prints.


### Device joined the network

```
INFO: ###### ===== JOINED EVENT ==== ######
```

### Alarm event occured, generating uplink
```
INFO: ###### ===== ALARM EVENT ==== ######
INFO: Request uplink
```

### Send done

```
INFO: ###### ===== TX DONE EVENT ==== ######
```
