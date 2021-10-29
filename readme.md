# ComputhermQThermostat Arduino controller

This is an ESPHome custom component which allows you to control a Computherm/Delta Q7RF/Q8RF receiver equiped furnace using a TI CC1101 transceiver module. This component defines a switch platform for state toggling and a service for pairing.

based on the work of denxhun, flogi, nistvan86
by Attila Farago, 2021

**Use this project at your own risk. Reporting and/or fixing issues is always welcome.**

## Hardware
You need a CC1101 module which is assembled for 868 MHz. The chip on its own can be configured for many targets, but the antenna design on the board have to be tuned for the specific frequency in mind.

The CC1101 module is connected to the standard SPI pins of ESP8266 (secondary SPI PINs, the first set is used by the SPI flash chip).

Connections:

    D1_MINI    RFM117W    RFM217W    DHT22       10K_Ohm_pullup_resistor
    =========================================================
    3.3V       VCC        VCC        VCC(pin1)   PIN2
    GND        GND        GND        GND(pin4)   -
    D1         -          DATA       -            -
    D2         DATA       -          -            -
    D3         -          -          DATA(pin2)  PIN1

## FEATURES
  Reads the state of Computherm Q thermostats throu RF and 
    publishes state via MQTT
  Accepts control message for virtual thermostats via MQTT /wifito868gw/computherm/DEVID/control
    sends control command to Computherm receiver via RF repeatedly
    flashes LED if virtual thermostat is ON
  Publishes config (currently readonly status) via MQTT /wifito868gw/computherm/DEVID/status
  Supports pairing of new virtual thermostat via MQTT /wifito868gw/computherm/DEVID/pair
  Reads connected DHT22 sensor
    publishes temp and humidity sensor state via MQTT /wifito868gw/sensor/status
  Publishes availability information via MQTT /wifito868gw/availability 
    upon WiFi disconnect publishes offline availability througth LWT
    
## HARDWARE
  ESP 8266 Wemos D1 mini
  Hope RFM117W-868S1 868MHz RF transmitter, ASK
  Hope RFM217W-868S1 868MHz, RF Receiver, OOK
  DHT22 Digital Temperature and Humidity Temperature Sensor

## PASSKEY
  it seens that the first 4 bytes are arbitrary
  the last byte 
    zone 1 --> 0111
    zone 2 --> 1011
    zone 3 --> 0011
    zone 4 --> 1101 (have not checked)

## CONFIGURATION
  thermo_id - passkeys of the thermostats 1..4
  thermo_readonly - you can only read physical thermostats and read and control virtual thermostats
  
## SOURCES
  https://ardu.blog.hu/2019/12/05/computherm_q8rf_hardver_es_szoftver_part
  https://github.com/denxhun/ComputhermRF
  https://flogi-diyiot.blog.hu/2021/10/13/rf868mhz_wifi_gateway_esp8266_rfm217w_rfm119w_computherm_q8rf
  https://flogi-diyiot.blog.hu/2021/10/15/rf868mhz_wifi_gateway_esp8266_rfm217w_rfm119w_computherm_q8rf_masodik_resz_adas
  https://chewett.co.uk/blog/1476/using-the-dht22-temperature-sensor-with-a-wemos-d1-mini-esp8266/
  https://computherm.info/sites/default/files/Q8RF-Manual-EN.pdf
