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

OLD_TINYUSB_INQUIRY = """    const char vid[] = "TinyUSB";
    const char pid[] = "TEST MSC Storage";
    const char rev[] = "0.1";"""

NEW_TINYUSB_INQUIRY = """    const char vid[] = "ZJU-P4";
    const char pid[] = "P4_BUOY TF";
    const char rev[] = "3.0";"""

OLD_TINYUSB_APP_HOOKS_ANCHOR = """// Invoked when received SCSI_CMD_INQUIRY
// Application fill vendor id, product id and revision with string up to 8, 16, 4 characters respectively"""

NEW_TINYUSB_APP_HOOKS = """__attribute__((weak)) void app_usb_msc_eject_cb(void) {}

// Invoked when received SCSI_CMD_INQUIRY
// Application fill vendor id, product id and revision with string up to 8, 16, 4 characters respectively"""

OLD_TINYUSB_EJECT_HOOK = """    if (load_eject && !start) {
        // Eject media from the storage
        msc_storage_mount_to_app();
    }"""

NEW_TINYUSB_EJECT_HOOK = """    if (load_eject && !start) {
        // Eject media from the storage
        msc_storage_mount_to_app();
        app_usb_msc_eject_cb();
    }"""

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

OLD_SDMMC_ISR_NULL_GUARD = """static void sd_host_isr(void *arg)
{
    sd_host_sdmmc_ctlr_t *ctlr = (sd_host_sdmmc_ctlr_t *)arg;
    sd_host_sdmmc_slot_t *slot = ctlr->slot[ctlr->cur_slot_id];

    sd_host_sdmmc_event_t event = {};"""

NEW_SDMMC_ISR_NULL_GUARD = """static void sd_host_isr(void *arg)
{
    sd_host_sdmmc_ctlr_t *ctlr = (sd_host_sdmmc_ctlr_t *)arg;
    sd_host_sdmmc_slot_t *slot = ctlr->slot[ctlr->cur_slot_id];

    if (slot == NULL) {
        uint32_t pending = sdmmc_ll_get_intr_status(ctlr->hal.dev);
        uint32_t dma_pending = sdmmc_ll_get_idsts_interrupt_raw(ctlr->hal.dev);
        sdmmc_ll_clear_interrupt(ctlr->hal.dev, pending);
        sdmmc_ll_clear_idsts_interrupt(ctlr->hal.dev, dma_pending);
        return;
    }

    sd_host_sdmmc_event_t event = {};"""

OLD_SDMMC_REMOVE_CURRENT_SLOT = """    if (ctlr->slot[slot_ctx->slot_id]) {
        slot_registered = true;
        ctlr->slot[slot_ctx->slot_id] = NULL;
        ctlr->registered_slot_nums -= 1;
    }
    portEXIT_CRITICAL(&ctlr->spinlock);"""

NEW_SDMMC_REMOVE_CURRENT_SLOT = """    if (ctlr->slot[slot_ctx->slot_id]) {
        slot_registered = true;
        ctlr->slot[slot_ctx->slot_id] = NULL;
        ctlr->registered_slot_nums -= 1;
        if (ctlr->cur_slot_id == slot_ctx->slot_id) {
            for (int slot_id = 0; slot_id < SOC_SDMMC_NUM_SLOTS; slot_id++) {
                if (ctlr->slot[slot_id] != NULL) {
                    ctlr->cur_slot_id = slot_id;
                    break;
                }
            }
        }
    }
    portEXIT_CRITICAL(&ctlr->spinlock);"""

OLD_FAT_FORMAT_REMOUNT = """    esp_err_t err = s_f_mount(card, s_ctx[id]->fs, pdrv, &s_ctx[id]->mount_config, NULL);
    if (err != ESP_OK) {
        unmount_card_core(base_path, card);
        ESP_LOGE(TAG, \"failed to format, resources recycled, please mount again\");
    }

    return ret;"""

OLD_HOSTED_STA_TX_ALLOC = """\t/*  Prepare transport buffer directly consumable */
\tcopy_buff = mempool_alloc(((struct mempool*)chan_arr[ESP_STA_IF]->memp), MAX_TRANSPORT_BUFFER_SIZE, true);
\tassert(copy_buff);
\tg_h.funcs->_h_memcpy(copy_buff+H_ESP_PAYLOAD_HEADER_OFFSET, buffer, len);"""

NEW_HOSTED_STA_TX_ALLOC = """\t/*  Prepare transport buffer directly consumable */
\tcopy_buff = mempool_alloc(((struct mempool*)chan_arr[ESP_STA_IF]->memp), MAX_TRANSPORT_BUFFER_SIZE, true);
\tif (!copy_buff) {
\t\terrno = -ENOBUFS;
\t\treturn ESP_ERR_NO_MEM;
\t}
\tg_h.funcs->_h_memcpy(copy_buff+H_ESP_PAYLOAD_HEADER_OFFSET, buffer, len);"""

OLD_HOSTED_AP_TX_ALLOC = """\t/*  Prepare transport buffer directly consumable */
\tcopy_buff = mempool_alloc(((struct mempool*)chan_arr[ESP_AP_IF]->memp), MAX_TRANSPORT_BUFFER_SIZE, true);
\tassert(copy_buff);
\tg_h.funcs->_h_memcpy(copy_buff+H_ESP_PAYLOAD_HEADER_OFFSET, buffer, len);"""

NEW_HOSTED_AP_TX_ALLOC = """\t/*  Prepare transport buffer directly consumable */
\tcopy_buff = mempool_alloc(((struct mempool*)chan_arr[ESP_AP_IF]->memp), MAX_TRANSPORT_BUFFER_SIZE, true);
\tif (!copy_buff) {
\t\terrno = -ENOBUFS;
\t\treturn ESP_ERR_NO_MEM;
\t}
\tg_h.funcs->_h_memcpy(copy_buff+H_ESP_PAYLOAD_HEADER_OFFSET, buffer, len);"""

NEW_FAT_FORMAT_REMOUNT = """    esp_err_t err = s_f_mount(card, s_ctx[id]->fs, pdrv, &s_ctx[id]->mount_config, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, \"failed to remount after format; resources retained for caller cleanup\");
        return err;
    }

    return ret;"""


def project_managed_components_dir(root: Path) -> Path:
    override = os.environ.get("IDF_PROJECT_MANAGED_COMPONENTS_PATH", "")
    if override:
        return Path(override).resolve()
    return root / "managed_components"


def patch_tinyusb_msc(root: Path) -> int:
    target = project_managed_components_dir(root) / "espressif__esp_tinyusb" / "tinyusb_msc.c"
    if not target.exists():
        print(f"component patch skipped; missing {target}")
        return 0

    text = target.read_text(encoding="utf-8")
    changed = False
    if NEW_TINYUSB_WRITE in text:
        print("component patch already applied: esp_tinyusb sync MSC write")
    elif OLD_TINYUSB_WRITE in text:
        text = text.replace(OLD_TINYUSB_WRITE, NEW_TINYUSB_WRITE, 1)
        changed = True
        print("component patch applied: esp_tinyusb sync MSC write")
    else:
        print(
            f"component patch failed: expected deferred write call not found in {target}",
            file=sys.stderr,
        )
        return 1

    if NEW_TINYUSB_INQUIRY in text:
        print("component patch already applied: esp_tinyusb product inquiry")
    elif OLD_TINYUSB_INQUIRY in text:
        text = text.replace(OLD_TINYUSB_INQUIRY, NEW_TINYUSB_INQUIRY, 1)
        changed = True
        print("component patch applied: esp_tinyusb product inquiry")
    else:
        print(
            f"component patch failed: expected MSC inquiry strings not found in {target}",
            file=sys.stderr,
        )
        return 1

    if NEW_TINYUSB_APP_HOOKS in text:
        print("component patch already applied: esp_tinyusb app eject hook declaration")
    elif OLD_TINYUSB_APP_HOOKS_ANCHOR in text:
        text = text.replace(OLD_TINYUSB_APP_HOOKS_ANCHOR, NEW_TINYUSB_APP_HOOKS, 1)
        changed = True
        print("component patch applied: esp_tinyusb app eject hook declaration")
    else:
        print(
            f"component patch failed: expected MSC inquiry anchor not found in {target}",
            file=sys.stderr,
        )
        return 1

    if NEW_TINYUSB_EJECT_HOOK in text:
        print("component patch already applied: esp_tinyusb app eject hook")
    elif OLD_TINYUSB_EJECT_HOOK in text:
        text = text.replace(OLD_TINYUSB_EJECT_HOOK, NEW_TINYUSB_EJECT_HOOK, 1)
        changed = True
        print("component patch applied: esp_tinyusb app eject hook")
    else:
        print(
            f"component patch failed: expected MSC start/stop eject block not found in {target}",
            file=sys.stderr,
        )
        return 1

    if changed:
        target.write_text(text, encoding="utf-8")
    return 0


def patch_esp_hosted_tx_no_assert(root: Path) -> int:
    target = (
        project_managed_components_dir(root)
        / "espressif__esp_hosted"
        / "host"
        / "drivers"
        / "transport"
        / "transport_drv.c"
    )
    if not target.exists():
        print(f"component patch skipped; missing {target}")
        return 0

    text = target.read_text(encoding="utf-8")
    changed = False
    if NEW_HOSTED_STA_TX_ALLOC in text:
        print("component patch already applied: esp_hosted STA TX no assert")
    elif OLD_HOSTED_STA_TX_ALLOC in text:
        text = text.replace(OLD_HOSTED_STA_TX_ALLOC, NEW_HOSTED_STA_TX_ALLOC, 1)
        changed = True
        print("component patch applied: esp_hosted STA TX no assert")
    else:
        print(
            f"component patch failed: expected esp_hosted STA TX allocation block not found in {target}",
            file=sys.stderr,
        )
        return 1

    if NEW_HOSTED_AP_TX_ALLOC in text:
        print("component patch already applied: esp_hosted AP TX no assert")
    elif OLD_HOSTED_AP_TX_ALLOC in text:
        text = text.replace(OLD_HOSTED_AP_TX_ALLOC, NEW_HOSTED_AP_TX_ALLOC, 1)
        changed = True
        print("component patch applied: esp_hosted AP TX no assert")
    else:
        print(
            f"component patch failed: expected esp_hosted AP TX allocation block not found in {target}",
            file=sys.stderr,
        )
        return 1

    if changed:
        target.write_text(text, encoding="utf-8")
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


def patch_idf_sdmmc_isr_null_guard() -> int:
    idf_value = os.environ.get("IDF_PATH", "")
    if not idf_value:
        print("IDF SDMMC ISR patch failed: IDF_PATH is not set", file=sys.stderr)
        return 1

    idf_path = Path(idf_value).resolve()
    version = read_idf_version(idf_path)
    if version != (6, 0, 1):
        print(
            f"IDF SDMMC ISR patch skipped: expected ESP-IDF 6.0.1, found {version}",
            file=sys.stderr,
        )
        return 1

    target = idf_path / "components" / "esp_driver_sdmmc" / "src" / "sd_host_sdmmc.c"
    if not target.exists():
        print(f"IDF SDMMC ISR patch failed: missing {target}", file=sys.stderr)
        return 1

    text = target.read_text(encoding="utf-8")
    if NEW_SDMMC_ISR_NULL_GUARD in text:
        print("IDF patch already applied: SDMMC removed-slot ISR guard")
        return 0
    if OLD_SDMMC_ISR_NULL_GUARD not in text:
        print(
            f"IDF SDMMC ISR patch failed: expected ESP-IDF 6.0.1 source not found in {target}",
            file=sys.stderr,
        )
        return 1

    target.write_text(
        text.replace(OLD_SDMMC_ISR_NULL_GUARD, NEW_SDMMC_ISR_NULL_GUARD, 1),
        encoding="utf-8",
    )
    print("IDF patch applied: SDMMC removed-slot ISR guard (upstream f408e1a8)")
    return 0


def patch_idf_sdmmc_removed_slot_owner() -> int:
    idf_value = os.environ.get("IDF_PATH", "")
    if not idf_value:
        print("IDF SDMMC slot owner patch failed: IDF_PATH is not set", file=sys.stderr)
        return 1

    idf_path = Path(idf_value).resolve()
    version = read_idf_version(idf_path)
    if version != (6, 0, 1):
        print(
            f"IDF SDMMC slot owner patch skipped: expected ESP-IDF 6.0.1, found {version}",
            file=sys.stderr,
        )
        return 1

    target = idf_path / "components" / "esp_driver_sdmmc" / "src" / "sd_host_sdmmc.c"
    if not target.exists():
        print(f"IDF SDMMC slot owner patch failed: missing {target}", file=sys.stderr)
        return 1

    text = target.read_text(encoding="utf-8")
    if NEW_SDMMC_REMOVE_CURRENT_SLOT in text:
        print("IDF patch already applied: SDMMC removed-slot owner handoff")
        return 0
    if OLD_SDMMC_REMOVE_CURRENT_SLOT not in text:
        print(
            f"IDF SDMMC slot owner patch failed: expected ESP-IDF 6.0.1 source not found in {target}",
            file=sys.stderr,
        )
        return 1

    target.write_text(
        text.replace(
            OLD_SDMMC_REMOVE_CURRENT_SLOT,
            NEW_SDMMC_REMOVE_CURRENT_SLOT,
            1,
        ),
        encoding="utf-8",
    )
    print("IDF patch applied: SDMMC removed-slot owner handoff")
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
    ret = patch_esp_hosted_tx_no_assert(root)
    if ret != 0:
        return ret
    ret = patch_idf_sdmmc_slot_cleanup()
    if ret != 0:
        return ret
    ret = patch_idf_sdmmc_isr_null_guard()
    if ret != 0:
        return ret
    ret = patch_idf_sdmmc_removed_slot_owner()
    if ret != 0:
        return ret
    return patch_idf_fat_format_remount()


if __name__ == "__main__":
    raise SystemExit(main())
