#!/usr/bin/env python3
"""Apply small, reproducible patches to IDF managed components."""

from pathlib import Path
import os
import re
import sys


OLD_TINYUSB_WRITE = (
    "esp_err_t err = msc_storage_write_sector_deferred(lun, lba, offset, bufsize, buffer);"
)
NEW_TINYUSB_WRITE = (
    "esp_err_t err = msc_storage_write_sector(lun, lba, offset, bufsize, buffer);"
)

OLD_SDMMC_SLOT_CLEANUP = """err:
    free(slot);

    return ret;
}"""

NEW_SDMMC_SLOT_CLEANUP = """err:
    portENTER_CRITICAL(&ctlr_ctx->spinlock);
    if (ctlr_ctx->slot[config->slot_id] == slot) {
        ctlr_ctx->slot[config->slot_id] = NULL;
        ctlr_ctx->registered_slot_nums -= 1;
    }
    portEXIT_CRITICAL(&ctlr_ctx->spinlock);
    free(slot);

    return ret;
}"""

OLD_FAT_FORMAT_REMOUNT = """    esp_err_t err = s_f_mount(card, s_ctx[id]->fs, pdrv, &s_ctx[id]->mount_config, NULL);
    if (err != ESP_OK) {
        unmount_card_core(base_path, card);
        ESP_LOGE(TAG, \"failed to format, resources recycled, please mount again\");
    }

    return ret;"""

NEW_FAT_FORMAT_REMOUNT = """    esp_err_t err = s_f_mount(card, s_ctx[id]->fs, pdrv, &s_ctx[id]->mount_config, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, \"failed to remount after format; resources retained for caller cleanup\");
        return err;
    }

    return ret;"""


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


def read_idf_version(idf_path: Path) -> tuple[int, int, int] | None:
    version_file = idf_path / "tools" / "cmake" / "version.cmake"
    if not version_file.exists():
        return None
    text = version_file.read_text(encoding="utf-8")
    values: list[int] = []
    for key in ("MAJOR", "MINOR", "PATCH"):
        match = re.search(rf"set\(IDF_VERSION_{key}\s+(\d+)\)", text)
        if not match:
            return None
        values.append(int(match.group(1)))
    return values[0], values[1], values[2]


def patch_idf_sdmmc_slot_cleanup() -> int:
    idf_value = os.environ.get("IDF_PATH", "")
    if not idf_value:
        print("IDF SDMMC patch failed: IDF_PATH is not set", file=sys.stderr)
        return 1

    idf_path = Path(idf_value).resolve()
    version = read_idf_version(idf_path)
    if version != (6, 0, 1):
        print(
            f"IDF SDMMC patch skipped: expected ESP-IDF 6.0.1, found {version}",
            file=sys.stderr,
        )
        return 1

    target = idf_path / "components" / "esp_driver_sdmmc" / "src" / "sd_host_sdmmc.c"
    if not target.exists():
        print(f"IDF SDMMC patch failed: missing {target}", file=sys.stderr)
        return 1

    text = target.read_text(encoding="utf-8")
    if NEW_SDMMC_SLOT_CLEANUP in text:
        print("IDF patch already applied: SDMMC add-slot rollback")
        return 0
    if OLD_SDMMC_SLOT_CLEANUP not in text:
        print(
            f"IDF SDMMC patch failed: expected ESP-IDF 6.0.1 source not found in {target}",
            file=sys.stderr,
        )
        return 1

    target.write_text(
        text.replace(OLD_SDMMC_SLOT_CLEANUP, NEW_SDMMC_SLOT_CLEANUP, 1),
        encoding="utf-8",
    )
    print("IDF patch applied: SDMMC add-slot rollback (upstream 09ff8795)")
    return 0


def patch_idf_fat_format_remount() -> int:
    idf_value = os.environ.get("IDF_PATH", "")
    if not idf_value:
        print("IDF FAT format patch failed: IDF_PATH is not set", file=sys.stderr)
        return 1

    idf_path = Path(idf_value).resolve()
    version = read_idf_version(idf_path)
    if version != (6, 0, 1):
        print(
            f"IDF FAT format patch skipped: expected ESP-IDF 6.0.1, found {version}",
            file=sys.stderr,
        )
        return 1

    target = idf_path / "components" / "fatfs" / "vfs" / "vfs_fat_sdmmc.c"
    if not target.exists():
        print(f"IDF FAT format patch failed: missing {target}", file=sys.stderr)
        return 1

    text = target.read_text(encoding="utf-8")
    if NEW_FAT_FORMAT_REMOUNT in text:
        print("IDF patch already applied: FAT format remount error propagation")
        return 0
    if OLD_FAT_FORMAT_REMOUNT not in text:
        print(
            f"IDF FAT format patch failed: expected ESP-IDF 6.0.1 source not found in {target}",
            file=sys.stderr,
        )
        return 1

    target.write_text(
        text.replace(OLD_FAT_FORMAT_REMOUNT, NEW_FAT_FORMAT_REMOUNT, 1),
        encoding="utf-8",
    )
    print("IDF patch applied: FAT format remount error propagation")
    return 0


def main() -> int:
    if len(sys.argv) != 2:
        print("usage: apply_component_patches.py <project-root>", file=sys.stderr)
        return 2

    root = Path(sys.argv[1]).resolve()
    ret = patch_tinyusb_msc(root)
    if ret != 0:
        return ret
    ret = patch_idf_sdmmc_slot_cleanup()
    if ret != 0:
        return ret
    return patch_idf_fat_format_remount()


if __name__ == "__main__":
    raise SystemExit(main())
