# Changelog

## v2.0.0

- Added USB HS OTG Mass Storage export for the whole TF card with exclusive ownership handoff.
- Added automatic `USB_EXPORT` entry on host enumeration plus manual `/api/mode/usb?confirm=USB`.
- Raised USB MSC SDMMC export speed to 40 MHz and apply a build-time `esp_tinyusb` MSC write patch for direct synchronous SDMMC writes.
- Added USB MSC watch, benchmark, and eject helper scripts.
- Added idle annotated AVI enrichment and source-aware recording metadata.
- Sanitized default Wi-Fi credentials for public GitHub publishing.
