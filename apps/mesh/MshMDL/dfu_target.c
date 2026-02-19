/** @file
 *  @brief Bluetooth Mesh DFU Target role handler
 */

/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "btstack/bluetooth.h"
#include "system/includes.h"
#include "bt_common.h"
#include "api/sig_mesh_api.h"
#include "model_api.h"
#include "adaptation.h"
#include "mesh_target_node_ota.h"
#include "dfu_target.h"

#define LOG_TAG             "[Mesh-DFU_Target]"
#define LOG_ERROR_ENABLE
#define LOG_DEBUG_ENABLE
#define LOG_INFO_ENABLE
/* #define LOG_DUMP_ENABLE */
#define LOG_CLI_ENABLE
#include "debug.h"

#if (CONFIG_BT_MESH_DFU_TARGET)

/**
 * @brief MCUboot image header representation for image version
 *
 * The header for an MCUboot firmware image contains an embedded
 * version number, in semantic versioning format. This structure
 * represents the information it contains.
 */
struct mcuboot_img_sem_ver {
    uint8_t major;
    uint8_t minor;
    uint16_t revision;
    uint32_t build_num;
};



struct target_img_t target_img;

static struct bt_mesh_blob_io_flash *blob_flash_stream;
static enum bt_mesh_dfu_effect img_effect = BT_MESH_DFU_EFFECT_NONE;

static struct bt_mesh_dfu_img dfu_imgs[] = { {
        .fwid = &((struct mcuboot_img_sem_ver) {}),
        .fwid_len = sizeof(struct mcuboot_img_sem_ver),
    }
};

#if (CONFIG_BT_MESH_DFU_METADATA)
static size_t flash_area_size_get(uint8_t area_id)
{
    const struct flash_area *area;
    size_t fa_size;
    int err;
    fa_size = 307200; //300K
    return fa_size;
}
#endif

static bool is_firmware_newer(struct mcuboot_img_sem_ver *new, struct mcuboot_img_sem_ver *cur)
{
    if (new->major > cur->major) {
        return true;
    } else if (new->major < cur->major) {
        return false;
    }

    if (new->minor > cur->minor) {
        return true;
    } else if (new->minor > cur->minor) {
        return false;
    }

    if (new->revision > cur->revision) {
        return true;
    } else if (new->revision > cur->revision) {
        return false;
    }

    //if (new->build_num > cur->build_num) {
    //return true;
    //}

    return false;
}

static int dfu_meta_check(struct bt_mesh_dfu_srv *srv,
                          const struct bt_mesh_dfu_img *img,
                          struct net_buf_simple *metadata_raw,
                          enum bt_mesh_dfu_effect *effect)
{
#if (CONFIG_BT_MESH_DFU_METADATA)
    struct mcuboot_img_sem_ver *img_ver = (struct mcuboot_img_sem_ver *) dfu_imgs[0].fwid;
    struct bt_mesh_dfu_metadata metadata;
    uint8_t key[16] = {};
    uint32_t hash;
    uint16_t temp_crc;
    int err;

    err = bt_mesh_dfu_metadata_decode(metadata_raw, &metadata);
    if (err) {
        log_error("Unable to decode metadata: %d", err);
        return -EINVAL;
    }

    log_info("Received firmware metadata:");
    log_info("\tVersion: %u.%u.%u+%u", metadata.fw_ver.major, metadata.fw_ver.minor,
             metadata.fw_ver.revision, metadata.fw_ver.build_num);
    log_info("\tSize: %u", metadata.fw_size);
    log_info("\tCore Type: 0x%x", metadata.fw_core_type);

    if (metadata.fw_core_type & BT_MESH_DFU_FW_CORE_TYPE_APP) {
        log_info("\tComposition data hash: 0x%x", metadata.comp_hash);
        log_info("\tElements: %u", metadata.elems);
    }

    if (metadata.user_data_len > 0) {
        size_t len;
        uint8_t user_data[2 * (CONFIG_BT_MESH_DFU_METADATA_MAXLEN - 18) + 1];

        len = bin2hex(metadata.user_data, metadata.user_data_len, user_data,
                      (sizeof(user_data) - 1));
        user_data[len] = '\0';
        log_info("\tUser data: %s", user_data);
    }

    log_info("\tUser data length: %u", metadata.user_data_len);

    if (!is_firmware_newer((struct mcuboot_img_sem_ver *) &metadata.fw_ver, img_ver)) {
        log_error("New firmware version is older");
        return -EINVAL;
    }

    target_img.fwid[0] = metadata.fw_ver.major;
    target_img.fwid[1] = metadata.fw_ver.minor;
    target_img.fwid[2] = (metadata.fw_ver.revision >> 8) & 0xFF;
    target_img.fwid[3] = metadata.fw_ver.revision & 0xFF;
    target_img.fwid[4] = 0x00;//(metadata.fw_ver.build_num >> 24) & 0xFF;
    target_img.fwid[5] = 0x00;//(metadata.fw_ver.build_num >> 16) & 0xFF;
    target_img.fwid[6] = 0x00;//(metadata.fw_ver.build_num >> 8) & 0xFF;
    target_img.fwid[7] = 0x00;//metadata.fw_ver.build_num & 0xFF;

    mesh_targe_ota_process(DFU_NODE_OTA_REQ, &(metadata.fw_ver.major), 0, 0, 0);

    if (!(metadata.fw_core_type & BT_MESH_DFU_FW_CORE_TYPE_APP)) {
        log_error("Only application core firmware is supported by the sample");
        return -EINVAL;
    }

    if (flash_area_size_get(0) < metadata.fw_size ||
        flash_area_size_get(1) < metadata.fw_size) {
        log_error("New firmware won't fit into flash.");
        return -EINVAL;
    }

    temp_crc = (metadata.fw_ver.build_num >> 16) & 0xFFFF;
    mesh_targe_ota_process(DFU_NODE_OTA_FILE_INFO, NULL, metadata.fw_size, temp_crc, 0);

    err = bt_mesh_dfu_metadata_comp_hash_local_get(key, &hash);
    if (err) {
        log_error("Failed to compute composition data hash: %d", err);
        return -EINVAL;
    }

    log_info("Current composition data hash: 0x%x", hash);

    if (hash == metadata.comp_hash) {
        img_effect = BT_MESH_DFU_EFFECT_NONE;
    } else {
        img_effect = BT_MESH_DFU_EFFECT_UNPROV;
    }

#else
    img_effect = BT_MESH_DFU_EFFECT_UNPROV;
#endif /* CONFIG_BT_MESH_DFU_METADATA */

    log_info("Metadata check succeeded, effect: %d", img_effect);

    *effect = img_effect;

    return 0;
}

static int dfu_start(struct bt_mesh_dfu_srv *srv,
                     const struct bt_mesh_dfu_img *img,
                     struct net_buf_simple *metadata,
                     const struct bt_mesh_blob_io **io)
{
    log_info("Firmware upload started");

    *io = &blob_flash_stream->io;

    return 0;
}

static void dfu_end(struct bt_mesh_dfu_srv *srv, const struct bt_mesh_dfu_img *img, bool success)
{
    log_info("Firmware upload %s", success ? "succeeded" : "failed");

    if (!success) {
        mesh_targe_ota_exit();
        return;
    }

    /* TODO: Add verification code here. */

    bt_mesh_dfu_srv_verified(srv);

    mesh_targe_ota_process(DFU_NODE_OTA_END, NULL, 0, 0, 0);
}

static int dfu_recover(struct bt_mesh_dfu_srv *srv,
                       const struct bt_mesh_dfu_img *img,
                       const struct bt_mesh_blob_io **io)
{
    log_info("Recovering the firmware upload");

    /* TODO: Need to recover the effect. */

#if (CONFIG_BT_MESH_DFD_SRV)
    return -ENOTSUP;
#else
    *io = &blob_flash_stream->io;

    return 0;
#endif
}

static void do_reboot(struct k_work *work)
{
    cpu_reset();
}

static int dfu_apply(struct bt_mesh_dfu_srv *srv, const struct bt_mesh_dfu_img *img)
{
    static struct k_work_delayable pending_reboot;

    log_info("Applying the new firmware");

    store_pending_target_cfg();

    if (img_effect == BT_MESH_DFU_EFFECT_UNPROV) {
        bt_mesh_reset();

        log_info("Pending the mesh settings to cleared before rebooting...");

        /* Let the mesh reset its settings before rebooting the device. */
        k_work_init_delayable(&pending_reboot, do_reboot);
        k_work_schedule(&pending_reboot, K_MSEC(1));
    } else {
        /* No need to unprovision device. Reboot immediately. */
        cpu_reset();
    }

    return 0;
}

static const struct bt_mesh_dfu_srv_cb dfu_handlers = {
    .check = dfu_meta_check,
    .start = dfu_start,
    .end = dfu_end,
    .apply = dfu_apply,
    .recover = dfu_recover,
};

struct bt_mesh_dfu_srv dfu_srv =
    BT_MESH_DFU_SRV_INIT(&dfu_handlers, dfu_imgs, ARRAY_SIZE(dfu_imgs));

extern struct file_parameter_t file_parameter;

static void image_version_load(void)
{
    struct mcuboot_img_sem_ver *img_ver = (struct mcuboot_img_sem_ver *) dfu_imgs[0].fwid;

    target_cfg_set();

    img_ver->major = target_img.fwid[0];
    img_ver->minor = target_img.fwid[1];
    img_ver->revision = (target_img.fwid[2] << 8) | target_img.fwid[3];
    img_ver->build_num = (target_img.fwid[4] << 24) | (target_img.fwid[5] << 16) | \
                         (target_img.fwid[6] << 8) | target_img.fwid[7];

    log_info("Current image version: %u.%u.%u+%u\n", img_ver->major, img_ver->minor,
             img_ver->revision, img_ver->build_num);
}

int dfu_target_init(struct bt_mesh_blob_io_flash *flash_stream)
{
    blob_flash_stream = flash_stream;

    image_version_load();

    return 0;
}

void dfu_target_image_confirm(void)
{
    int err = 0;

    // err = boot_write_img_confirmed(); //check the image for update.
    if (err) {
        log_error("Failed to confirm image: %d\n", err);
    }

    /* Switch DFU Server state to the Idle state if it was in the Applying state. */
    bt_mesh_dfu_srv_applied(&dfu_srv);
}
#endif
