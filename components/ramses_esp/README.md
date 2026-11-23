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
      type: local
      path: components
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
Basic/M146 M5-Bus SPI pins and routes CSN to GPIO25; set the M146 DIP switches
to match that CSN route. GPIO25 is also used by the Core Basic speaker, so do
not enable a speaker component on that pin while using this example. Verify the
M146 DIP configuration and the node's existing SPI bus before using it on
hardware.

At setup, the component resets the CC1101, logs PARTNUM/VERSION, writes the
existing RAMSES 868.3 MHz register and PA profiles, and checks a register
readback. It leaves the radio idle for later frame/UART work.
