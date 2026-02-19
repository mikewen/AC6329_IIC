/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "btstack/bluetooth.h"
#include "system/includes.h"
#include "bt_common.h"
#include "api/sig_mesh_api.h"
#include "model_api.h"
#include "mesh_dfu_app_cmd_protocol.h"

#define LOG_TAG             "[Mesh-DFU_Dist]"
#define LOG_ERROR_ENABLE
#define LOG_DEBUG_ENABLE
#define LOG_INFO_ENABLE
/* #define LOG_DUMP_ENABLE */
#define LOG_CLI_ENABLE
#include "debug.h"

#if (CONFIG_BT_MESH_DFU_DIST)

static struct bt_mesh_blob_io_flash *blob_flash_stream;

static int hex2char(uint8_t x, char *c)
{
    if (x <= 9) {
        *c = x + (char)'0';
    } else  if (x <= 15) {
        *c = x - 10 + (char)'a';
    } else {
        return -EINVAL;
    }

    return 0;
}

static size_t bin2hex(const uint8_t *buf, size_t buflen, char *hex, size_t hexlen)
{
    if (hexlen < (buflen * 2U + 1U)) {
        return 0;
    }

    for (size_t i = 0; i < buflen; i++) {
        if (hex2char(buf[i] >> 4, &hex[2U * i]) < 0) {
            return 0;
        }
        if (hex2char(buf[i] & 0xf, &hex[2U * i + 1U]) < 0) {
            return 0;
        }
    }

    hex[2U * buflen] = '\0';
    return 2U * buflen;
}

static void slot_info_print(const struct bt_mesh_dfu_slot *slot, const u8_t *idx)
{
    char fwid[2 * CONFIG_BT_MESH_DFU_FWID_MAXLEN + 1];
    char metadata[2 * CONFIG_BT_MESH_DFU_METADATA_MAXLEN + 1];
    size_t len;

    len = bin2hex(slot->fwid, slot->fwid_len, fwid, sizeof(fwid));
    fwid[len] = '\0';
    len = bin2hex(slot->metadata, slot->metadata_len, metadata,
                  sizeof(metadata));
    metadata[len] = '\0';

    if (idx != NULL) {
        log_info("Slot %u:", *idx);
    } else {
        log_info("Slot:");
    }
    log_info("\tSize:     %u bytes", slot->size);
    log_info("\tFWID:     %s", fwid);
    log_info("\tMetadata: %s", metadata);
}

static int dfd_srv_recv(struct bt_mesh_dfd_srv *srv,
                        const struct bt_mesh_dfu_slot *slot,
                        const struct bt_mesh_blob_io **io)
{
    log_info("Uploading new firmware image to the distributor.");
    slot_info_print(slot, NULL);

    *io = &blob_flash_stream->io;

    return 0;
}

static void dfd_srv_del(struct bt_mesh_dfd_srv *srv,
                        const struct bt_mesh_dfu_slot *slot)
{
    log_info("Deleting the firmware image from the distributor.");
    slot_info_print(slot, NULL);
}

static int dfd_srv_send(struct bt_mesh_dfd_srv *srv,
                        const struct bt_mesh_dfu_slot *slot,
                        const struct bt_mesh_blob_io **io)
{
    log_info("Starting the firmware distribution.");
    slot_info_print(slot, NULL);

    *io = &blob_flash_stream->io;

    return 0;
}

static const char *dfd_phase_to_str(enum bt_mesh_dfd_phase phase)
{
    switch (phase) {
    case BT_MESH_DFD_PHASE_IDLE:
        return "Idle";
    case BT_MESH_DFD_PHASE_TRANSFER_ACTIVE:
        return "Transfer Active";
    case BT_MESH_DFD_PHASE_TRANSFER_SUCCESS:
        return "Transfer Success";
    case BT_MESH_DFD_PHASE_APPLYING_UPDATE:
        return "Applying Update";
    case BT_MESH_DFD_PHASE_COMPLETED:
        return "Completed";
    case BT_MESH_DFD_PHASE_FAILED:
        return "Failed";
    case BT_MESH_DFD_PHASE_CANCELING_UPDATE:
        return "Canceling Update";
    case BT_MESH_DFD_PHASE_TRANSFER_SUSPENDED:
        return "Transfer Suspended";
    default:
        return "Unknown Phase";
    }
}

static void dfd_srv_phase(struct bt_mesh_dfd_srv *srv, enum bt_mesh_dfd_phase phase)
{
    log_info("Distribution phase changed to %s", dfd_phase_to_str(phase));

    set_dfu_distributor_state(phase, 0x00);
}

static struct bt_mesh_dfd_srv_cb dfd_srv_cb = {
    .recv = dfd_srv_recv,
    .del = dfd_srv_del,
    .send = dfd_srv_send,
    .phase = dfd_srv_phase,
};

struct bt_mesh_dfd_srv dfd_srv = BT_MESH_DFD_SRV_INIT(&dfd_srv_cb);

void dfu_distributor_init(struct bt_mesh_blob_io_flash *flash_stream)
{
    blob_flash_stream = flash_stream;
}
#endif
