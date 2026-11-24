# `ramses_esp` ESPHome component

The RX phase configures the CC1101 on an ESPHome SPI bus, starts the upstream
RAMSES asynchronous GFSK receive mode, and decodes received RAMSES frames from
ESPHome UART input. It validates Manchester coding, message layout, payload
length, checksum, and frame trailer. It does not add TX or MQTT.

The component requires an ESPHome `spi:` bus with MOSI and MISO, a separate CS
pin, and an ESPHome `uart:` bus configured for 38400 baud, 8 data bits, no
parity, and one stop bit. It attaches to the SPI bus through ESPHome's
SPI-device API and reads bytes through ESPHome's UART component. If the node
already defines either bus, reference its ID instead of defining it again.

```yaml
external_components:
  - source:
      type: git
      url: https://github.com/VinnyEatWorld/ramses_esp
      ref: esphome-m5stack-core
    components: [ramses_esp]

spi:
  id: ramses_spi
  clk_pin: <existing SCLK pin>
  mosi_pin: <existing MOSI pin>
  miso_pin: <existing MISO pin>

uart:
  id: ramses_uart
  rx_pin: <GPIO connected to CC1101 GDO2>
  baud_rate: 38400
  data_bits: 8
  parity: NONE
  stop_bits: 1
  rx_full_threshold: 1

ramses_esp:
  id: ramses_gateway_radio
  spi_id: ramses_spi
  uart_id: ramses_uart
  cs_pin: <GPIO selected for CC1101 CSN>

sensor:
  - platform: ramses_esp
    ramses_esp_id: ramses_gateway_radio
    type: co2
    name: Ventilation CO2
  - platform: ramses_esp
    ramses_esp_id: ramses_gateway_radio
    type: boost_duration
    name: Requested boost duration

text_sensor:
  - platform: ramses_esp
    ramses_esp_id: ramses_gateway_radio
    type: fan_mode
    name: Ventilation mode

binary_sensor:
  - platform: ramses_esp
    ramses_esp_id: ramses_gateway_radio
    type: co2_sensor_presence
    name: CO2 sensor presence
```

RAMSES IDs are deliberately absent from the public configuration. The explicit
binding flow will learn the fan ID from the exchange, then associate the CO2
sensor and remote from traffic addressed to that fan. Until binding support is
implemented and completed, the filtered telemetry entities remain without a
state. `sensor_timeout` is seconds (default 1800) and controls when the CO2
presence binary sensor changes to off after the last valid `1298` or paired
`31E0` message. The boost entity reports the duration requested by a remote; it
is not a confirmed countdown from the fan.

The example at `examples/ramses_esp_m5stack_core.yaml` uses the M5Stack Core
Basic/M146 M5-Bus SPI pins and routes CSN to GPIO15. The M5Stack M146 schematic
maps SW1-1/2/3/4 to CSN candidate pins GPIO25/15/12/0 and SW2-1/2/3 to GDO2
GPIO35/13/5 and SW2-4/5/6 to GDO0 GPIO35/13/5. The example selects SW1-2,
SW2-1 and SW2-5. The official product pin map confirms SPI on M5-Bus pins 7/9/11
and the DIP-routed CSN/GDO signals on the switchable pins.

GPIO15 is a strapping pin on the classic ESP32. ESPHome emits a configuration
warning when a known ESP32 strapping GPIO is selected for CSN. This warning is
informational and does not establish that boot will fail or succeed. The M146
schematic shows a 10 kOhm pull-up on the GPIO0 branch, but does not specify a
dedicated pull-up on the GPIO15 branch. The ESP32's internal strapping bias and
the CC1101 CSN input suggest no intentional low drive from the module, but the
reset level should be measured on the assembled Core Basic + M146. For a
deployment where that electrical check is unavailable, select a non-strapping
CSN route and set the corresponding SW1 switch instead.

At setup, the component applies the upstream CSN reset pulse (1 µs high, 10 µs
low, then 41 µs high) through ESPHome's SPI-device CS control, sends `SRES`, and
waits at most 100 ms for CHIP_RDYn, logs PARTNUM/VERSION and MARCSTATE, writes the existing RAMSES 868.3 MHz
register and PA profiles, and checks the frequency, modem, packet, sync, and
AGC configuration by register readback. Register operations follow the
upstream two-byte SPI transaction shape with separate TX and RX buffers. It
then applies the upstream asynchronous RX settings (`IOCFG0=0x2E`,
`PKTCTRL0=0x32`), flushes the RX FIFO, starts RX, and verifies `MARCSTATE=RX`.
The ESPHome main loop drains UART bytes non-blockingly, scans for the upstream
RAMSES sync sequence `00 33 55 53`, applies the same Manchester nibble mapping
and message-field order as `components/frame/frame.c` and
`components/message/message.c`, and accepts a frame only when the message is
complete, its 8-bit byte sum including checksum is zero, and the `0x35` trailer
arrives. Every accepted frame logs message type, all three address slots,
opcode, payload length, and payload bytes. Rejected frames log the reason;
five-second debug summaries include byte, sync, valid-frame, and rejected-frame
counters. Parsing is limited to the upstream `MAX_RAW` and `MAX_PAYLOAD`
bounds. This phase still does not transmit or publish MQTT.

The RF profile is inherited from `components/cc1101/cc1101_param.c` (also
vendored as `cc1101_param.c` in this external component). It sets FREQ2/1/0 to
`0x21/0x65/0x6A` (868.3 MHz with the CC1101 26 MHz crystal), `MDMCFG4/3` to
`0x6A/0x83` (about 38.383 kbit/s), `MDMCFG2` to `0x10` (GFSK, no hardware sync),
`DEVIATN` to `0x50`, and FIFO packet mode values as listed in that source.
These are upstream project settings, not newly calculated values. The current
RX phase listens for the raw demodulated stream; it does not transmit or
publish MQTT.
