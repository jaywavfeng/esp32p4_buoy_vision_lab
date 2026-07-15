# Changelog

## v3.0.1

- Hardened TF mounting, write/read verification, I/O fault latching and customer-facing recovery without routine reboot loops.
- Fixed camera/JPEG/ISP cleanup so repeated wake and standby cycles recover after initialization failures.
- Made validation jobs asynchronous and kept active Web, stream, download, validation and storage work from being interrupted by automatic FIELD entry.
- Made configuration changes strict, transactional and POST-only; router passwords are no longer accepted in URLs or echoed by APIs.
- Added persistent Web controls for automatic FIELD entry, idle countdown and recording segment duration.
- Fixed multi-interface Web client accounting and made stream statistics concurrency-safe.
- Made recording cleanup a coalesced background job with progress, deterministic residue verification and actionable Web errors; downloads and storage mutations now have race-free admission.
- Improved AVI finalization, paired raw/annotated recovery, USB ownership recovery and ESP-IDF storage cleanup patches.

## v3.0.0

- Reworked the customer Web flow around one recording segment per row, with raw video, annotated video and manual fill-frame action grouped together.
- Added customer runtime settings for segment duration up to 14400 seconds, idle FIELD timeout, router credentials, network mode and model selection.
- Added Fish31/TinyCNN/COCO model switching, defaulting to Fish31 and applying the saved model to FIELD recording and manual enrichment.
- Fixed FIELD recording so raw and annotated AVI files are generated as a pair with matching frame count and duration.
- Removed idle automatic enrichment; enrichment now only runs after the user clicks a recording's fill-frame action.
- Added full recording cleanup from the Web UI and API.
- Changed client counting to track recent Web clients across Ethernet, AP and STA, preventing unintended auto FIELD entry while a wired client is open.
- Improved live preview startup by waiting for camera wake and recovering transient video open failures.
- Improved USB MSC export so inserting USB automatically exposes the TF card as `P4_BUOY` while Web remains online, then remounts TF after safe eject and unplug.
- Cleaned old experimental model artifacts and refreshed customer/developer documentation.

## v2.0.0

- Added USB HS OTG Mass Storage export for the whole TF card with exclusive ownership handoff.
- Added automatic `USB_EXPORT` entry on host enumeration plus manual `/api/mode/usb?confirm=USB`.
- Raised USB MSC SDMMC export speed to 40 MHz and applied a build-time `esp_tinyusb` MSC write patch for direct synchronous SDMMC writes.
- Added USB MSC watch, benchmark and eject helper scripts.
- Added annotated AVI enrichment and source-aware recording metadata.
- Sanitized default Wi-Fi credentials for public GitHub publishing.
