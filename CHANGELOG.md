# Changelog

## v2.1.0

- Added Fish31 MobileNetV3-Small board model as the default `fish31` route.
- Kept TinyCNN Marine 6-class classification and COCO YOLO11n detection as the other two main validation routes.
- Added board-verified Fish31/TinyCNN demo images, video validation assets, and classification-aware visualization.
- Documented 5-minute ESP32-P4 board benchmark results for Fish31, TinyCNN, and COCO.
- Removed Coke/Sprite sample gallery from the main validation path while keeping legacy source for reference.
- Sanitized public release docs, paths, and board-test host interface examples for GitHub publishing.

## v2.0.0

- Added USB HS OTG Mass Storage export for the whole TF card with exclusive ownership handoff.
- Added automatic `USB_EXPORT` entry on host enumeration plus manual `/api/mode/usb?confirm=USB`.
- Raised USB MSC SDMMC export speed to 40 MHz and apply a build-time `esp_tinyusb` MSC write patch for direct synchronous SDMMC writes.
- Added USB MSC watch, benchmark, and eject helper scripts.
- Added idle annotated AVI enrichment and source-aware recording metadata.
- Sanitized default Wi-Fi credentials for public GitHub publishing.
