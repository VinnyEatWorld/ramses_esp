# `ramses_esp` ESPHome component

Phase 1 adds CC1101 initialization on an ESPHome SPI bus. It does not start
radio RX/TX, install an IDF SPI bus, configure the radio UART, or publish MQTT.

The component requires an ESPHome `spi:` bus with MOSI and MISO and a separate
CS pin. It attaches to that bus through ESPHome's SPI-device API. If the node
already defines the SPI bus, set `spi_id` to its ID instead of defining it
again.

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

ramses_esp:
  spi_id: ramses_spi
  cs_pin: <GPIO selected for CC1101 CSN>
```

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
waits at most 100 ms for CHIP_RDYn,
logs PARTNUM/VERSION and MARCSTATE, writes the existing RAMSES 868.3 MHz
register and PA profiles, and checks the frequency, modem, packet, sync, and
AGC configuration by register readback. Register operations follow the
upstream two-byte SPI transaction shape with separate TX and RX buffers. It
verifies MARCSTATE is IDLE and leaves the radio there for later frame/UART work.

The RF profile is inherited from `components/cc1101/cc1101_param.c` (also
vendored as `cc1101_param.c` in this external component). It sets FREQ2/1/0 to
`0x21/0x65/0x6A` (868.3 MHz with the CC1101 26 MHz crystal), `MDMCFG4/3` to
`0x6A/0x83` (about 38.383 kbit/s), `MDMCFG2` to `0x10` (GFSK, no hardware sync),
`DEVIATN` to `0x50`, and FIFO packet mode values as listed in that source.
These are upstream project settings, not newly calculated or hardware-verified
values. The present phase configures them but does not start RF reception or
transmission.
