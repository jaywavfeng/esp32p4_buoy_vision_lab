#!/usr/bin/env python3
"""Apply small, reproducible patches to IDF managed components."""

from pathlib import Path
import sys


OLD_TINYUSB_WRITE = (
    "esp_err_t err = msc_storage_write_sector_deferred(lun, lba, offset, bufsize, buffer);"
)
NEW_TINYUSB_WRITE = (
    "esp_err_t err = msc_storage_write_sector(lun, lba, offset, bufsize, buffer);"
)


def patch_tinyusb_msc(root: Path) -> int:
    target = root / "managed_components" / "espressif__esp_tinyusb" / "tinyusb_msc.c"
    if not target.exists():
        print(f"component patch skipped; missing {target}")
        return 0

    text = target.read_text(encoding="utf-8")
    if NEW_TINYUSB_WRITE in text:
        print("component patch already applied: esp_tinyusb sync MSC write")
        return 0
    if OLD_TINYUSB_WRITE not in text:
        print(
            f"component patch failed: expected deferred write call not found in {target}",
            file=sys.stderr,
        )
        return 1

    target.write_text(text.replace(OLD_TINYUSB_WRITE, NEW_TINYUSB_WRITE, 1), encoding="utf-8")
    print("component patch applied: esp_tinyusb sync MSC write")
    return 0


def main() -> int:
    if len(sys.argv) != 2:
        print("usage: apply_component_patches.py <project-root>", file=sys.stderr)
        return 2

    root = Path(sys.argv[1]).resolve()
    return patch_tinyusb_msc(root)


if __name__ == "__main__":
    raise SystemExit(main())
