/*************************************************************************************************/
/*!
*  \file       mesh_dfu_app_cmd_protocol.c
*
*  \brief      distributor and app cmd protocol
*
*  Copyright (c) 2011-2022 ZhuHai Jieli Technology Co.,Ltd.
*
*  Licensed under the Apache License, Version 2.0 (the "License");
*  you may not use this file except in compliance with the License.
*  You may obtain a copy of the License at
*
*      http://www.apache.org/licenses/LICENSE-2.0
*
*  Unless required by applicable law or agreed to in writing, software
*  distributed under the License is distributed on an "AS IS" BASIS,
*  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*  See the License for the specific language governing permissions and
*  limitations under the License.
*/
/*************************************************************************************************/
#include "btstack/bluetooth.h"
#include "system/includes.h"
#include "api/sig_mesh_api.h"
#include "adaptation.h"
#include "mesh_dfu_app_cmd_protocol.h"
#include "mesh_distributor_loader.h"
#include "dfd_srv_internal.h"
#include "board_config.h"

#define LOG_TAG             "[Mesh_dfu_app]"
#define LOG_INFO_ENABLE
//#define LOG_DEBUG_ENABLE
#define LOG_ERROR_ENABLE
//#define LOG_DUMP_ENABLE
#define LOG_CLI_ENABLE
#include "mesh_log.h"

#if (CONFIG_MESH_MODEL == SIG_MESH_DFU_DISTRIBUTOR_DEMO)

/**************************************************************************************************
Macros
**************************************************************************************************/
#define CMD_HEADER_FLAG     0x5A

/* total length of Head flag, Length, Flag, OP, SN, CRC */
#define CMD_PROTOCOL_LENGTH 6

#define COMMAND_PACKET      0x81

#define NO_RESPONSE_PACKET  0x01

#define RESPONSE_PACKET     0x00

#define BUFFER_SIZE_TARGET  4096

/**************************************************************************************************
Data Types
**************************************************************************************************/



/**************************************************************************************************
Global Variables
**************************************************************************************************/
static opcode_state_t dfu_app_cmd;
static push_data_t push_data;
struct file_parameter_t file_parameter;

static uint8_t *global_buffer = NULL;
static size_t global_buffer_size = 0;
static size_t global_buffer_used = 0;
static uint32_t global_file_size = 0;

static uint8_t dfu_cfg_block_data[200];

static const struct bt_mesh_model *mod;

static uint8_t dist_state;
static uint8_t dist_val;

extern u16 hci_get_conn_handle(void);

/**************************************************************************************************
Functions
**************************************************************************************************/
void get_mesh_dfu_adv_filter(u8 *len, u8 **data)
{
    u8 adv_filter[] = {0x04, 0x16, 0x00, 0xBD, 0x01};
    u8 adv_len = 0x5;

    memcpy((*data) + (*len), adv_filter, adv_len);
    *len += adv_len;
}

void clear_sn(void)
{
    dfu_app_cmd.valid = false;

    log_info(">>> %s", __func__);
}

static bool check_sn(opcode_state_t *op, uint8_t sn)
{
    if (!op->valid) {
        /* If the op was not previously recorded, it is considered valid (initialized) */
        op->prev_sn = sn;
        op->valid = true;
        return true;
    }

    /* Check the serial number for legitimacy */
    if (sn == (op->prev_sn + 1) % 256) {
        /* Sequence number is legal, update prev_sn  */
        op->prev_sn = sn;
        return true;
    }

    /* Sequence number is not legal, could be a retransmission or an attack */
    return false;
}

static void send_request_data_action(void)
{
    struct send_action_packet_t request_data;
    static uint8_t i = 0;

    request_data.head_flag = 0x5A5A;
    request_data.field.length = 11;
    request_data.field.flag = 0x81;
    request_data.field.op = MESH_DFU_APP_OP_FILE_TRANS;
    request_data.field.sn = i++;
    request_data.field.action = ACTION_REQUEST_DATA;
    request_data.field.parameter_len = 0x04;

    if (file_parameter.file_size > 4096) {
        request_data.field.parameter = 4096;
    } else {
        request_data.field.parameter = file_parameter.file_size;
    }

    request_data.crc = CRC16(&(request_data.field), 11);



    int err = mesh_gatt_notify(hci_get_conn_handle(), ATT_CHARACTERISTIC_bd00_bd02_VALUE_HANDLE, &request_data, 15);

    log_info(">>> %s need file size %d notify_err 0x%x", __func__, request_data.field.parameter, err);
    log_info_hexdump(&request_data, sizeof(request_data));
}

static void send_stop_trans_action(void)
{
    struct stop_trans_action_packet_t request_data;
    static uint8_t i = 0;

    request_data.head_flag = 0x5A5A;
    request_data.field.length = 8;
    request_data.field.flag = 0x81;
    request_data.field.op = MESH_DFU_APP_OP_FILE_TRANS;
    request_data.field.sn = i++;
    request_data.field.action = ACTION_STOP_TRANS;
    request_data.field.parameter_len = 0x01;

    log_debug(">>> push_data crc 0x%x file_parameter crc 0x%x", push_data.crc, file_parameter.crc);

    if (push_data.crc == file_parameter.crc) {
        request_data.field.parameter = ACTION_OPERATION_SUCCESS;
        store_pending_dist_cfg();
    } else {
        request_data.field.parameter = ACTION_CRC_CHECK_FAILED;
    }

    request_data.crc = CRC16(&(request_data.field), 8);

    int err = mesh_gatt_notify(hci_get_conn_handle(), ATT_CHARACTERISTIC_bd00_bd02_VALUE_HANDLE, &request_data, 12);

    log_info(">>> %s parameter 0x%x notify_err %d", __func__, request_data.field.parameter, err);
    log_info_hexdump(&request_data, sizeof(request_data));
}

static uint8_t file_trans_cmd_response(void)
{
    struct response_action_packet_t response;
    uint8_t ret;

    response.head_flag = 0x5A5A;
    response.field.length = 0x9;
    response.field.flag = 0x00;
    response.field.op = MESH_DFU_APP_OP_FILE_TRANS;
    response.field.sn = dfu_app_cmd.prev_sn;
    response.field.state = OPERATION_SUCCESS;
    response.field.action = ACTION_TRANS_FILE;
    response.field.response_len = 0x1;

    if (file_parameter.file_size > CONFIG_RESERVED_AREA1_LEN) {
        response.field.response = ACTION_MEMORY_LACKED;
        ret = -1;
    } else {
        response.field.response = ACTION_OPERATION_SUCCESS;
        ret = 0;
    }

    response.crc = CRC16(&(response.field), 9);



    int err = mesh_gatt_notify(hci_get_conn_handle(), ATT_CHARACTERISTIC_bd00_bd02_VALUE_HANDLE, &response, 13);

    log_info(">>> %s notify_err %d", __func__, err);
    log_info_hexdump(&response, sizeof(response));

    return ret;
}

static void set_file_parameter(const uint8_t *data)
{
    clear_dist_cfg();

    memcpy(&(file_parameter.file_version), &data[0], 8);
    memcpy(&(file_parameter.file_size), &data[8], 4);
    memcpy(&(file_parameter.crc), &data[12], 2);
    //file_parameter.crc = (data[8] << 8) | data[9];

    global_file_size = file_parameter.file_size;

    log_info("file_size %d crc 0x%x", file_parameter.file_size, file_parameter.crc);
    log_info(">>> file version %s ", bt_hex(file_parameter.file_version, 8));

    if (file_trans_cmd_response()) {
        log_error("file size is large!!");
        return;
    }

    mesh_dist_loader_init(W_TYPE);

    /* Send file trans cmd for request data */
    sys_timeout_add(NULL, send_request_data_action, 1000);
}

struct file_parameter_t *get_file_parameter(void)
{
    return &file_parameter;
}

static void calculate_crc(void)
{
    uint16_t crc;
    size_t size;

    mesh_dist_loader_calculate_crc(&crc, &size, file_parameter.file_size);
    push_data.crc = crc;

    if (file_parameter.file_size == size) {
        log_info(">>> read from flash: pus_data.crc 0x%x file size %d", push_data.crc, file_parameter.file_size);
    } else {
        log_error(">>> file size read from flash failed.");
    }

    sys_timeout_add(NULL, send_stop_trans_action, 500);
}

static void handle_push_data(const uint8_t *data)
{
    uint8_t parameter_len;
    uint16_t temp_offset;
    uint16_t ret;

    parameter_len = data[0];
    memcpy(&temp_offset, &data[1], 2);

    push_data.buff_offset += temp_offset;

    log_debug("parameter_len %d temp_offset %d ", parameter_len, temp_offset);

    // 如果全局缓冲区为空或不足以存储更多数据，则重新分配
    if (global_buffer == NULL) {
        if (global_buffer != NULL) {
            free(global_buffer); // 释放旧缓冲区
        }

        global_buffer = malloc(BUFFER_SIZE_TARGET);
        if (global_buffer == NULL) {
            // 处理内存分配失败
            log_error("buffer malloc failed.");
            return;
        }
        global_buffer_used = 0; // 重置已使用大小
    }

    // 读取数据到缓冲区
    global_buffer_used += (parameter_len - 4);
    memcpy(global_buffer + temp_offset, &data[5], (parameter_len - 4));
    log_debug(">>> global_buffer_used %d ", global_buffer_used);
    //log_info_hexdump((global_buffer+temp_offset), (parameter_len-4));

    // 如果缓冲区已满或达到目标大小，则写入Flash并重置缓冲区状态
    if (global_buffer_used >= BUFFER_SIZE_TARGET) {
        // mesh_dist_loader_write_data(global_buffer, BUFFER_SIZE_TARGET);
        ret = mesh_dist_loader_write_data(global_buffer, global_buffer_used);
        log_info(">>> write buffer %d remain file size %d", ret, file_parameter.file_size);
        global_buffer_used = 0; // 重置已使用大小，而不是释放和重新分配缓冲区（除非需要）
        sys_timeout_add(NULL, send_request_data_action, 200);
    }

    global_file_size -= (parameter_len - 4);
    log_debug("file_size %d ", global_file_size);
    if (global_file_size == 0) {  // 如果缓冲区中还有剩余数据（不足4K），也可以写入Flash（可选）
        ret = mesh_dist_loader_write_data(global_buffer, global_buffer_used);
        log_info(">>> write data done, write buffer %d", ret);
        mesh_dist_loader_read_complete();
        sys_timeout_add(NULL, calculate_crc, 500);
        free(global_buffer);
    }
}

static void handle_file_trans_cmd(const uint8_t *data, size_t length)
{
    uint8_t action;

    action = data[1];

    switch (action) {
    case ACTION_TRANS_FILE:
        set_file_parameter(&data[3]);
        break;

    case ACTION_PUSH_DATA:
        handle_push_data(&data[2]);
        break;

    default:
        break;
    }

}

static void dfu_cmd_response(MESH_DFU_APP_OPCODE opcode, opcode_state_t cmd, uint8_t state)
{
    cmd_response_packet_t response;

    response.head_flag = 0x5A5A;
    response.field.length = 0x6;
    response.field.flag = 0x00;
    response.field.op = opcode;
    response.field.sn = cmd.prev_sn;
    response.field.state = state;
    response.crc = CRC16(&(response.field), 6);


    int err = mesh_gatt_notify(hci_get_conn_handle(), ATT_CHARACTERISTIC_bd00_bd02_VALUE_HANDLE, &response, 10);

    log_info(">>> %s state 0x%x notify_err %d ", __func__, state, err);
    log_info_hexdump(&response, sizeof(response));
}

static void handle_add_slot_cmd(const uint8_t *data, size_t length)
{
    struct bt_mesh_dfu_slot *slot;
    u32 size;
    u16 crc;
    uint8_t fwid[CONFIG_BT_MESH_DFU_FWID_MAXLEN];
    size_t fwid_len, metadata_len;
    uint8_t metadata[CONFIG_BT_MESH_DFU_METADATA_MAXLEN];
    int err;

    file_parameter.file_size = (uint32_t)data[11] << 16 | (uint32_t)data[10] << 8 | data[9];
    log_info("Adding slot (size: %u)", file_parameter.file_size);

    slot = bt_mesh_dfu_slot_reserve();
    if (!slot) {
        log_error("Failed to reserve slot.");
        return;
    }

    fwid_len = 8;
    for (int i = 0; i < fwid_len; i++) {
        fwid[i] = data[fwid_len - i];
    }

    log_debug(">>> fwid");
    log_info_hexdump(fwid, 8);

    bt_mesh_dfu_slot_fwid_set(slot, fwid, fwid_len);

    metadata_len = 18;
    /* firmware version */
    memcpy(&metadata[0], fwid, 8);
    metadata[6] = file_parameter.crc;
    metadata[7] = (file_parameter.crc >> 8);
    /* firmware size */
    metadata[8] = data[9];
    metadata[9] = data[10];
    metadata[10] = data[11];
    /* firmware core type */
    metadata[11] = data[12];//BT_MESH_DFU_FW_CORE_TYPE_APP
    /* composition data hash */
    metadata[15] = data[16];
    metadata[14] = data[15];
    metadata[13] = data[14];
    metadata[12] = data[13];
    /* elements */
    metadata[16] = data[17];
    metadata[17] = data[18];

    log_debug(">>> metadata");
    log_info_hexdump(metadata, 18);

    bt_mesh_dfu_slot_info_set(slot,  file_parameter.file_size, metadata, metadata_len);

    err = bt_mesh_dfu_slot_commit(slot);
    if (err) {
        log_error("Failed to commit slot: %d", err);
        bt_mesh_dfu_slot_release(slot);
        dfu_cmd_response(MESH_DFU_APP_OP_ADD_SLOT, dfu_app_cmd, OPERATION_FAILED);
        return;
    }

    log_info("Slot added. Index: %u", bt_mesh_dfu_slot_img_idx_get(slot));

    dfu_cmd_response(MESH_DFU_APP_OP_ADD_SLOT, dfu_app_cmd, OPERATION_SUCCESS);
}

static bool bt_mesh_shell_mdl_first_get(uint16_t id, const struct bt_mesh_model **mod)
{
    const struct bt_mesh_comp *comp = bt_mesh_comp_get();

    for (int i = 0; i < comp->elem_count; i++) {
        *mod = bt_mesh_model_find(&comp->elem[i], id);
        if (*mod) {
            return true;
        }
    }

    return false;
}

static void handle_add_target_cmd(const uint8_t *data, size_t length)
{
    log_info(">>> %s ", __func__);


    if (!mod && !bt_mesh_shell_mdl_first_get(BT_MESH_MODEL_ID_DFD_SRV, &mod)) {
        log_error("no this model.");
        return;
    }

    struct bt_mesh_dfd_srv *dfd_srv = mod->rt->user_data;

    if (bt_mesh_dfu_cli_is_busy(&dfd_srv->dfu)) {
        log_error("{\"status\": %d, \"target_cnt\": %d}", BT_MESH_DFD_ERR_BUSY_WITH_DISTRIBUTION, dfd_srv->target_cnt);
        return;
    }

    uint16_t target_addr;
    uint8_t img_idx;
    enum bt_mesh_dfd_status status;

    for (int i = 0; i < data[1]; i++) {
        memcpy(&target_addr, &data[2 + (3 * i)], 2);
        img_idx = data[4 + (3 * i)];

        log_info(">>> target addr 0x%x, img_idx 0x%x", target_addr, img_idx);

        status = bt_mesh_dfd_srv_receiver_add(dfd_srv, target_addr, img_idx);
        if (status != BT_MESH_DFD_SUCCESS) {
            log_error("{\"status\": %d, \"target_cnt\": %d}", status, dfd_srv->target_cnt);
            dfu_cmd_response(MESH_DFU_APP_OP_ADD_TARGET, dfu_app_cmd, OPERATION_FAILED);
            return;
        }
    }

    log_info("{\"status\": %d, \"target_cnt\": %d}", BT_MESH_DFD_SUCCESS, dfd_srv->target_cnt);

    dfu_cmd_response(MESH_DFU_APP_OP_ADD_TARGET, dfu_app_cmd, OPERATION_SUCCESS);
}

static void handle_dfu_start_cmd(const uint8_t *data, size_t length)
{
    log_info(">>> %s ", __func__);

    if (!mod && !bt_mesh_shell_mdl_first_get(BT_MESH_MODEL_ID_DFD_SRV, &mod)) {
        log_error("no this model.");
        return;
    }

    struct bt_mesh_dfd_srv *dfd_srv = mod->rt->user_data;
    struct bt_mesh_dfd_start_params params;
    int err = 0;

    // params.app_idx = 0;
    memcpy(&params.app_idx, &data[1], 2);
    // params.slot_idx = 0;
    memcpy(&params.slot_idx, &data[3], 2);
    // params.group = BT_MESH_ADDR_UNASSIGNED;
    memcpy(&params.group, &data[5], 2);

    params.apply = data[7];
    params.ttl = data[8];
    params.timeout_base = 1U;
    params.xfer_mode = BT_MESH_BLOB_XFER_MODE_PUSH;

    log_info_hexdump(&params, sizeof(params));

    enum bt_mesh_dfd_status status = bt_mesh_dfd_srv_start(dfd_srv, &params);

    log_info("{ \"status\": %d, \"phase\": %d,", status, dfd_srv->phase);

    if (dfd_srv->phase != BT_MESH_DFD_PHASE_IDLE && dfd_srv->dfu.xfer.slot) {
        log_info("  \"group\": %d, \"app_idx\": %d, "
                 "\"ttl\": %d, \"timeout_base\": %d, \"xfer_mode\": %d, "
                 "\"apply\": %d, \"slot_idx\": %d", dfd_srv->inputs.group,
                 dfd_srv->inputs.app_idx, dfd_srv->inputs.ttl, dfd_srv->inputs.timeout_base,
                 dfd_srv->dfu.xfer.blob.mode, dfd_srv->apply, dfd_srv->slot_idx);
    }
    log_info(" }");

    dfu_cmd_response(MESH_DFU_APP_OP_START, dfu_app_cmd, (status != BT_MESH_DFD_SUCCESS ? OPERATION_FAILED : OPERATION_SUCCESS));
}

static void handle_dfu_apply_cmd(const uint8_t *data, size_t length)
{
    log_info(">>> %s ", __func__);

    if (!mod && !bt_mesh_shell_mdl_first_get(BT_MESH_MODEL_ID_DFD_SRV, &mod)) {
        return;
    }

    struct bt_mesh_dfd_srv *dfd_srv = mod->rt->user_data;

    enum bt_mesh_dfd_status status = bt_mesh_dfd_srv_apply(dfd_srv);

    log_info("{ \"status\": %d, \"phase\": %d,", status, dfd_srv->phase);

    if (dfd_srv->phase != BT_MESH_DFD_PHASE_IDLE && dfd_srv->dfu.xfer.slot) {
        log_info("  \"group\": %d, \"app_idx\": %d, "
                 "\"ttl\": %d, \"timeout_base\": %d, \"xfer_mode\": %d, "
                 "\"apply\": %d, \"slot_idx\": %d", dfd_srv->inputs.group,
                 dfd_srv->inputs.app_idx, dfd_srv->inputs.ttl, dfd_srv->inputs.timeout_base,
                 dfd_srv->dfu.xfer.blob.mode, dfd_srv->apply, dfd_srv->slot_idx);
    }

    log_info(" }");

    dfu_cmd_response(MESH_DFU_APP_OP_APPLY, dfu_app_cmd, (status != BT_MESH_DFD_SUCCESS ? OPERATION_FAILED : OPERATION_SUCCESS));
}

static void handle_dfu_cancel_cmd(const uint8_t *data, size_t length)
{
    log_info(">>> %s ", __func__);

    if (!mod && !bt_mesh_shell_mdl_first_get(BT_MESH_MODEL_ID_DFD_SRV, &mod)) {
        return;
    }

    struct bt_mesh_dfd_srv *dfd_srv = mod->rt->user_data;

    enum bt_mesh_dfd_status status = bt_mesh_dfd_srv_cancel(dfd_srv, NULL);

    log_info("{ \"status\": %d, \"phase\": %d", status, dfd_srv->phase);

    if (dfd_srv->phase != BT_MESH_DFD_PHASE_IDLE && dfd_srv->dfu.xfer.slot) {
        log_info(", \"group\": %d, \"app_idx\": %d, "
                 "\"ttl\": %d, \"timeout_base\": %d, \"xfer_mode\": %d, "
                 "\"apply\": %d, \"slot_idx\": %d", dfd_srv->inputs.group,
                 dfd_srv->inputs.app_idx, dfd_srv->inputs.ttl, dfd_srv->inputs.timeout_base,
                 dfd_srv->dfu.xfer.blob.mode, dfd_srv->apply, dfd_srv->slot_idx);
    }

    log_info(" }");

    dfu_cmd_response(MESH_DFU_APP_OP_DELETE_TARGET, dfu_app_cmd, (status != BT_MESH_DFD_SUCCESS ? OPERATION_FAILED : OPERATION_SUCCESS));
}

static void handle_delete_target_cmd(const uint8_t *data, size_t length)
{
    log_info(">>> %s ", __func__);

    if (!mod && !bt_mesh_shell_mdl_first_get(BT_MESH_MODEL_ID_DFD_SRV, &mod)) {
        return;
    }

    struct bt_mesh_dfd_srv *dfd_srv = mod->rt->user_data;

    enum bt_mesh_dfd_status status = bt_mesh_dfd_srv_receivers_delete_all(
                                         dfd_srv);

    log_info("{\"status\": %d, \"target_cnt\": %d}", status, dfd_srv->target_cnt);

    dfu_cmd_response(MESH_DFU_APP_OP_DELETE_TARGET, dfu_app_cmd, (status != BT_MESH_DFD_SUCCESS ? OPERATION_FAILED : OPERATION_SUCCESS));
}

static void read_dfu_cfg_cmd_response(opcode_state_t cmd)
{

    struct read_dfu_cfg_response_packet_t response;

    response.head_flag = 0x5A5A;
    response.field.length = 22;
    response.field.flag = 0x00;
    response.field.op = MESH_DFU_APP_OP_READ_DFU_CFG;
    response.field.sn = cmd.prev_sn;
    response.field.state = OPERATION_SUCCESS;
    response.field.action = ACTION_READ_CFG;
    response.field.response_len = 14;
    //response.field.file_version = file_parameter.file_version;
    memcpy(response.field.file_version, file_parameter.file_version, 8);
    response.field.file_size = file_parameter.file_size;
    response.field.file_crc = file_parameter.crc;
    response.crc = CRC16(&(response.field), 22);

    log_info(">>> file version %s ", bt_hex(response.field.file_version, 8));

    int err = mesh_gatt_notify(hci_get_conn_handle(), ATT_CHARACTERISTIC_bd00_bd02_VALUE_HANDLE, &response, (2 + 2 + 22));

    log_debug(">>> %s notify_err %d ", __func__, err);
    log_info_hexdump(&response, sizeof(response));
}

static void handle_read_dfu_config_cmd(const uint8_t *data, size_t length)
{
    log_info(">>> %s ", __func__);

    read_dfu_cfg_cmd_response(dfu_app_cmd);
}

static void get_dfu_distributor_state(uint8_t *state, uint8_t *value)
{
    *state = dist_state;
    *value = dist_val;
}

void set_dfu_distributor_state(uint8_t state, uint8_t val)
{
    dist_state = state;
    dist_val = val;

    notify_dfu_distributor_state();
}

static void dfu_distributor_state_response(opcode_state_t cmd)
{
    struct dist_state_response_packet_t response;

    response.head_flag = 0x5A5A;
    response.field.length = 10;
    response.field.flag = 0x00;
    response.field.op = MESH_DFU_APP_OP_READ_DFU_STATE;
    response.field.sn = cmd.prev_sn;
    response.field.state = OPERATION_SUCCESS;
    response.field.type = 0x01;
    response.field.parameter_len = 2;
    get_dfu_distributor_state(&(response.field.dist_phase), &(response.field.dist_state));
    response.crc = CRC16(&(response.field), 10);

    int err = mesh_gatt_notify(hci_get_conn_handle(), ATT_CHARACTERISTIC_bd00_bd02_VALUE_HANDLE, &response, (4 + 10));

    log_info(">>> dist phase 0x%x state 0x%x notify_err %d", response.field.dist_phase, response.field.dist_state, err);
    log_info_hexdump(&response, sizeof(response));
}

static void dfu_target_state_analyze(const uint8_t *data, opcode_state_t cmd)
{
    uint8_t len;
    uint8_t target_start;
    uint8_t target_end;
    size_t i, j;
    uint16_t crc;

    len = data[0];
    target_start = data[1];
    target_end = data[2];

    if (!mod && !bt_mesh_shell_mdl_first_get(BT_MESH_MODEL_ID_DFD_SRV, &mod)) {
        log_error("no this model.");
        return;
    }

    struct bt_mesh_dfd_srv *dfd_srv = mod->rt->user_data;

    log_info(">>> target count %d target_start %d target_end %d", dfd_srv->target_cnt, target_start, target_end);

    size_t total_size = sizeof(struct target_state_response_packet_t) + (dfd_srv->target_cnt * 4 * sizeof(uint8_t)) + 2 * sizeof(uint8_t);

    struct target_state_response_packet_t *response = (struct target_state_response_packet_t *)malloc(total_size);

    if (!response) {
        log_error("Failed to allocate memory");
        return;
    }

    response->head_flag = 0x5A5A;
    response->length = 9 + (dfd_srv->target_cnt * 4);
    response->flag = 0x00;
    response->op = MESH_DFU_APP_OP_READ_DFU_STATE;
    response->sn = cmd.prev_sn;
    response->state = OPERATION_SUCCESS;
    response->type = 0x02;
    response->response_len = 1 + (dfd_srv->target_cnt * 4);
    response->num = dfd_srv->target_cnt;

    for (i = 0, j = 0; i < dfd_srv->target_cnt; i++) {
        response->target[j] = dfd_srv->targets[i].blob.addr & 0xFF;
        response->target[j + 1] = (dfd_srv->targets[i].blob.addr >> 8) & 0xFF;
        if (dfd_srv->targets[i].phase == 0x02) {
            response->target[j + 2] = dfd_srv->targets[i].phase;
            response->target[j + 3] = bt_mesh_dfu_cli_progress(&dfd_srv->dfu);
            log_debug(">>> progress %d ", response->target[j + 3]);
        } else {
            response->target[j + 2] = dfd_srv->targets[i].phase;
            response->target[j + 3] = dfd_srv->targets[i].status;
        }

        j += 4;
        log_debug(">>> j %d addr 0x%x phase 0x%x", j, dfd_srv->targets[i].blob.addr, dfd_srv->targets[i].phase);
    }

    crc = CRC16(&(response->length), (9 + (dfd_srv->target_cnt * 4)));

    response->target[j] = crc & 0xFF;
    response->target[j + 1] = (crc >> 8) & 0xFF;



    int err = mesh_gatt_notify(hci_get_conn_handle(), ATT_CHARACTERISTIC_bd00_bd02_VALUE_HANDLE, response, (4 + 9 + (dfd_srv->target_cnt * 4)));

    log_debug(">>> %s total_size %d notify_err %d", __func__, total_size, err);
    log_info_hexdump(response, total_size);

    free(response);
}

static void handle_read_dfu_state_cmd(const uint8_t *data, size_t length)
{
    log_info(">>> %s ", __func__);

    if (data[1] == 0x01) {
        dfu_distributor_state_response(dfu_app_cmd);
    } else if (data[1] == 0x02) {
        dfu_target_state_analyze(&data[2], dfu_app_cmd);
    }
}

void notify_dfu_distributor_state(void)
{
    struct dist_state_notify_packet_t response;
    static i = 0;
    int err;

    response.head_flag = 0x5A5A;
    response.field.length = 9;
    response.field.flag = 0x01;
    response.field.op = MESH_DFU_APP_OP_NOTIFY_DFU_STATE;
    response.field.sn = i++;
    response.field.type = 0x01;
    response.field.parameter_len = 2;
    get_dfu_distributor_state(&(response.field.dist_phase), &(response.field.dist_state));
    response.crc = CRC16(&(response.field), 9);

    err = mesh_gatt_notify(hci_get_conn_handle(), ATT_CHARACTERISTIC_bd00_bd02_VALUE_HANDLE, &response, (4 + 9));

    log_info(">>> %s phase 0x%x state 0x%x err %d", __func__, response.field.dist_phase, response.field.dist_state, err);
    log_info_hexdump(&response, sizeof(response));
}

void notify_dfu_target_state(uint16_t addr, uint8_t phase, uint8_t state)
{
    struct target_state_notify_packet_t response;
    static i = 0;

    response.head_flag = 0x5A5A;
    response.field.length = 11;
    response.field.flag = 0x01;
    response.field.op = MESH_DFU_APP_OP_NOTIFY_DFU_STATE;
    response.field.sn = i++;
    response.field.type = 0x02;
    response.field.parameter_len = 4;
    response.field.target_addr = addr;
    response.field.target_phase = phase;
    response.field.target_state = state;
    response.crc = CRC16(&(response.field), 11);

    int err = mesh_gatt_notify(hci_get_conn_handle(), ATT_CHARACTERISTIC_bd00_bd02_VALUE_HANDLE, &response, (4 + 11));

    log_info(">>> %s addr 0x%x phase 0x%x state 0x%x notify_err %d", __func__, addr, phase, state, err);
    log_info_hexdump(&response, sizeof(response));
}

void parse_packet(const uint8_t *data, size_t length)
{
    uint16_t crc_value;
    uint16_t cal_crc;
    uint8_t  flag;
    uint8_t  opcode;
    uint8_t  sn;


    /* The minimum length needs to contain Head flag, Length, Flag, OP, SN  */
    if (length < CMD_PROTOCOL_LENGTH) {
        log_error("Packets too short to parse!!");
        return;
    }

    /* check header flag */
    if ((data[0] != CMD_HEADER_FLAG) || (data[1] != CMD_HEADER_FLAG)) {
        log_error("Header flag mismatched!!");
        return;
    }

    /* CRC Verification */
    memcpy(&crc_value, (&data[length - 2]), 2);
    cal_crc = CRC16((&data[2]), (length - 4));
    if (crc_value != cal_crc) {
        log_error("CRC verification failed, crc_value: 0x%x cal_crc 0x%x", crc_value, cal_crc);
        return;
    }

    /* Read Flag, OP, SN */
    flag = data[4];
    opcode = data[5];


    if ((flag == COMMAND_PACKET) || (flag == NO_RESPONSE_PACKET)) {

        if (!check_sn(&dfu_app_cmd, data[6])) {
            log_error("SN is not legal, could be a retransmission or an attack");
            return;
        }

        switch (opcode) {
        case MESH_DFU_APP_OP_FILE_TRANS:
            handle_file_trans_cmd(&data[6], (length - 4));
            break;

        case MESH_DFU_APP_OP_ADD_SLOT:
            handle_add_slot_cmd(&data[6], (length - 4));
            break;

        case MESH_DFU_APP_OP_ADD_TARGET:
            handle_add_target_cmd(&data[6], (length - 4));
            break;

        case MESH_DFU_APP_OP_START:
            handle_dfu_start_cmd(&data[6], (length - 4));
            break;

        case MESH_DFU_APP_OP_APPLY:
            handle_dfu_apply_cmd(&data[6], (length - 4));
            break;

        case MESH_DFU_APP_OP_CANCEL:
            handle_dfu_cancel_cmd(&data[6], (length - 4));
            break;

        case MESH_DFU_APP_OP_DELETE_TARGET:
            handle_delete_target_cmd(&data[6], (length - 4));
            break;

        case MESH_DFU_APP_OP_READ_DFU_CFG:
            handle_read_dfu_config_cmd(&data[6], (length - 4));
            break;

        case MESH_DFU_APP_OP_READ_DFU_STATE:
            handle_read_dfu_state_cmd(&data[6], (length - 4));
            break;

        default:
            break;
        }
    } else if (flag == RESPONSE_PACKET) {
        log_info("response packet.");
    }
}
#endif /* CONFIG_MESH_MODEL == SIG_MESH_DFU_DISTRIBUTOR_DEMO */
