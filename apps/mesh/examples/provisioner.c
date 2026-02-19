/*************************************************************************************************/
/*!
*  \file       provisioner.c
*
*  \brief      Main file for example
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
#include "bt_common.h"
#include "api/sig_mesh_api.h"
#include "model_api.h"
#include "adaptation.h"
#include "net.h"
#include "provisioner_config.h"
#include "foundation.h"
#include "feature_correct.h"
#include "os/os_api.h"

#define LOG_TAG             "[Provisioner_demo]"
#define LOG_ERROR_ENABLE
#define LOG_DEBUG_ENABLE
#define LOG_INFO_ENABLE
/* #define LOG_DUMP_ENABLE */
#define LOG_CLI_ENABLE
#include "debug.h"

#if (CONFIG_MESH_MODEL == SIG_MESH_PROVISIONER)

/**************************************************************************************************
Macros
**************************************************************************************************/
/* @brief Config current node features(Relay/Proxy/Friend/Low Power) */
#define BT_MESH_FEAT_SUPPORTED_TEMP         ( \
                                                0 \
                                            )
/* @brief Conifg complete local name */
#define BLE_DEV_NAME        'P', 'r', 'o', 'v', 'i', 's', 'i', 'o', 'n', 'e', 'r'

/* @brief Conifg MAC of current demo */
#define CUR_DEVICE_MAC_ADDR         		0x112233440000

/* @brief Conifg MAC of provisionee */
#define PROVISIONEE_DEVICE_MAC_ADDR         0x112233445566

/* Company Identifiers (see Bluetooth Assigned Numbers) */
#define BT_COMP_ID_LF           0x05D6// Zhuhai Jieli technology Co.,Ltd

/* Model Operation Codes */
#define BT_MESH_MODEL_OP_GEN_ONOFF_SET			BT_MESH_MODEL_OP_2(0x82, 0x02)
#define BT_MESH_MODEL_OP_GEN_ONOFF_SET_UNACK	BT_MESH_MODEL_OP_2(0x82, 0x03)
#define BT_MESH_MODEL_OP_GEN_ONOFF_STATUS		BT_MESH_MODEL_OP_2(0x82, 0x04)


/**************************************************************************************************
Data Types
**************************************************************************************************/
typedef struct {
    u16_t net_idx;
    u16_t addr;
    u8_t num_elem;
    u8_t config_step;
    u8_t config_failed_cnt;
    u8_t uuid[16];
    u8_t onoff_state;
} __prov_node;

/**************************************************************************************************
Function declaration
**************************************************************************************************/
static void provisioner_node_configuration(void);
static void gen_onoff_status(struct bt_mesh_model *model,
                             struct bt_mesh_msg_ctx *ctx,
                             struct net_buf_simple *buf);
static void provisioner_node_added(u16_t net_idx, u8_t uuid[16], u16_t addr, u8_t num_elem);
static void get_unprov_beacon(u8_t uuid[16],
                              bt_mesh_prov_oob_info_t oob_info,
                              u32_t *uri_hash);


/**************************************************************************************************
Global Variables
**************************************************************************************************/
/*
 * Publication Declarations
 *
 * The publication messages are initialized to the
 * the size of the opcode + content
 *
 * For publication, the message must be in static or global as
 * it is re-transmitted several times. This occurs
 * after the function that called bt_mesh_model_publish() has
 * exited and the stack is no longer valid.
 *
 * Note that the additional 4 bytes for the AppMIC is not needed
 * because it is added to a stack variable at the time a
 * transmission occurs.
 *
 */
BT_MESH_MODEL_PUB_DEFINE(gen_onoff_pub_cli, NULL, 2 + 2);

static const uint16_t net_idx;
static const uint16_t app_idx;

const int config_bt_mesh_features = BT_MESH_FEAT_SUPPORTED;

static u8_t flags;
static struct bt_mesh_comp prov_comp;
static uint8_t unprov_uuid[16];

static __prov_node curr_pair_node;

static struct bt_mesh_cfg_cli cfg_cli = {};

const uint8_t mesh_name[] = {
    BYTE_LEN(BLE_DEV_NAME) + 1, BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME, BLE_DEV_NAME,
};

static const u8_t match_dev_uuid[16] = {MAC_TO_LITTLE_ENDIAN(PROVISIONEE_DEVICE_MAC_ADDR)};

const u8_t prov_net_key[16] = {
    0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef,
    0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef,
};

const u8_t prov_dev_key[16] = {
    0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef,
    0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef,
};

const u8_t prov_app_key[16] = {
    0x06, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef,
    0x06, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef,
};

static const struct bt_mesh_model_op gen_onoff_cli_op[] = {
    { BT_MESH_MODEL_OP_GEN_ONOFF_STATUS, 1, gen_onoff_status },
    BT_MESH_MODEL_OP_END,
};

/* Element 0 Root Models */
static struct bt_mesh_model root_models[] = {
    BT_MESH_MODEL_CFG_SRV,
    BT_MESH_MODEL_CFG_CLI(&cfg_cli), // default for self-configuration network
    BT_MESH_MODEL(BT_MESH_MODEL_ID_GEN_ONOFF_CLI, gen_onoff_cli_op, &gen_onoff_pub_cli, NULL),
};

/* Root and Secondary Element Declarations */
static struct bt_mesh_elem elements[] = {
    BT_MESH_ELEM(0, root_models, BT_MESH_MODEL_NONE),
};

static const struct bt_mesh_comp composition = {
    .cid = BT_COMP_ID_LF,
    .elem = elements,
    .elem_count = ARRAY_SIZE(elements),
};

static const u8_t dev_uuid[16] = {MAC_TO_LITTLE_ENDIAN(CUR_DEVICE_MAC_ADDR)};

static const struct bt_mesh_prov prov = {
    .uuid = dev_uuid,
#if 0
    .output_size = 6,
    .output_actions = (BT_MESH_DISPLAY_NUMBER | BT_MESH_DISPLAY_STRING),
    .output_number = output_number,
    .output_string = output_string,
#else
    .output_size = 0,
    .output_actions = 0,
    .output_number = 0,
#endif
    .complete = prov_complete,
    .reset = prov_reset,
    .node_added = provisioner_node_added,
    .unprovisioned_beacon = get_unprov_beacon,
};

/**************************************************************************************************
Functions
**************************************************************************************************/
void get_mesh_adv_name(u8 *len, u8 **data)
{
    *len = sizeof(mesh_name);

    *data = mesh_name;
}

static void gen_onoff_status(struct bt_mesh_model *model,
                             struct bt_mesh_msg_ctx *ctx,
                             struct net_buf_simple *buf)
{
    u8_t	state;

    state = buffer_pull_u8_from_head(buf);

    log_info("Node 0x%04x OnOff status from 0x%04x with state 0x%02x\n",
             bt_mesh_model_elem(model)->rt->addr, ctx->addr, state);
}

/**
 * @brief Retransmission after failure to send a configuration message
 *
 */
static void config_msg_resend(void)
{
    if (curr_pair_node.config_failed_cnt < 3) {
        curr_pair_node.config_failed_cnt++;
        sys_timeout_add(NULL, provisioner_node_configuration, 500);
    } else {
        if (curr_pair_node.config_step < 2) {
            curr_pair_node.config_step++;
        } else {
            return;
        }

        curr_pair_node.config_failed_cnt = 0;
        sys_timeout_add(NULL, provisioner_node_configuration, 500);
    }
}

static void onoff_cli_publish(u16_t addr)
{
    int err;
    struct bt_mesh_model *mod_cli;
    struct bt_mesh_model_pub *pub_cli;

    mod_cli = &root_models[2];
    pub_cli = mod_cli->pub;

    pub_cli->addr = addr;

    log_info("publish to 0x%04x onoff_state 0x%x\n", pub_cli->addr, curr_pair_node.onoff_state);

    bt_mesh_model_msg_init(pub_cli->msg,
                           BT_MESH_MODEL_OP_GEN_ONOFF_SET);

    buffer_add_u8_at_tail(pub_cli->msg, curr_pair_node.onoff_state);
    buffer_add_u8_at_tail(pub_cli->msg, 0x06);
    curr_pair_node.onoff_state = !(curr_pair_node.onoff_state);
    err = bt_mesh_model_publish(mod_cli);
    if (err) {
        log_info("bt_mesh_model_publish err %d\n", err);
    }
}

/**
 * @brief Node configuration state machine, step by step node configuration.
 *
 */
static void provisioner_node_configuration(void)
{
    int err = 0;
    static uint8_t status;

    log_info("provisioner_node_configuration, step:%d", curr_pair_node.config_step);
    switch (curr_pair_node.config_step) {
    case 0:
        err = bt_mesh_cfg_cli_app_key_add(curr_pair_node.net_idx, curr_pair_node.addr, curr_pair_node.net_idx, 0, prov_app_key, NULL);
        config_msg_resend();
        break;

    case 1:
        err = bt_mesh_cfg_cli_mod_app_bind(curr_pair_node.net_idx, curr_pair_node.addr, curr_pair_node.addr, 0, BT_MESH_MODEL_ID_GEN_ONOFF_SRV, NULL);
        config_msg_resend();
        break;

    case 2:
        onoff_cli_publish(curr_pair_node.addr);
        config_msg_resend();
        break;

    default:
        break;
    }
}
/**
 * @brief New node provisioning complete callback, start node configuration.
 *
 */
static void provisioner_node_added(u16_t net_idx, u8_t uuid[16], u16_t addr, u8_t num_elem)
{
    log_info("Add new node!");

    log_info("addr:0x%04x", addr);
    log_info("net_idx:0x%x", net_idx);
    log_info("num_elem:%d", num_elem);
    log_info("uuid:");
    put_buf(uuid, 16);

    curr_pair_node.net_idx = net_idx;
    curr_pair_node.addr = addr;
    curr_pair_node.num_elem = num_elem;
    memcpy(curr_pair_node.uuid, uuid, 16);

    curr_pair_node.config_step = 0;
    curr_pair_node.config_failed_cnt = 0;
    curr_pair_node.onoff_state = 1;
    provisioner_node_configuration();
}

/**
 * @brief Scan to unprovisioned beacon callbacks
 *
 */
static void get_unprov_beacon(u8_t uuid[16],
                              bt_mesh_prov_oob_info_t oob_info,
                              u32_t *uri_hash)
{
    log_info("get_unprov_beacon, uuid:%s", bt_hex(uuid, 16));
    memcpy(unprov_uuid, uuid, 16);
}

static u8_t show_node(struct bt_mesh_cdb_node *node,
                      void *user_data)
{
    log_info("\n[node addr = 0x%x]\n", node->addr);
    log_info("uuid = %s\n", bt_hex(node->uuid, 6));
    log_info("net_idx = 0x%x\n", node->net_idx);
    log_info("num_elem = %d\n", node->num_elem);
    log_info("dev_key = %s\n", bt_hex(node->dev_key.key, 16));

    return 1;
}

/**
 * @brief Show all provisioned devices
 *
 */
static void Provisioner_show_cdb_node(void)
{
    log_info("Provisioner_show_cdb_node\n");

    bt_mesh_cdb_node_foreach(show_node, NULL);
}

static u8_t del_node(struct bt_mesh_cdb_node *node,
                     void *user_data)
{
    log_info("\n<Del> [node addr = 0x%x]\n", node->addr);

    bt_mesh_cdb_node_del(node, true);

    return 1;
}

/**
 * @brief Clearing provisioned node records
 *
 */
static void clear_store_node(void)
{
    log_info("clear_store_node");
    // clear_rpl();
    bt_mesh_cdb_node_foreach(del_node, NULL);
}

static u8_t get_store_node_num(struct bt_mesh_cdb_node *node,
                               void *user_data)
{
    *(u8_t *)user_data += 1;
    return 1;
}

static void provisioner_reset()
{
    bt_mesh_reset();
    p33_soft_reset();
}

void input_key_handler(u8 key_status, u8 key_number)
{
    log_info("key_number=0x%x", key_number);

    switch (key_status) {
    case KEY_EVENT_CLICK:
        log_info("  [KEY_EVENT_CLICK]  ");
        break;
    case KEY_EVENT_LONG:
        log_info("  [KEY_EVENT_LONG]  ");
        break;
    case KEY_EVENT_HOLD:
        log_info("  [KEY_EVENT_HOLD]  ");
        break;
    default :
        return;
    }
    if (key_status == KEY_EVENT_LONG) {
        switch (key_number) {
        case 0:
            break;
        case 2:
            log_info("\n  <bt_mesh_reset> \n");
            sys_timeout_add(NULL, provisioner_reset, 1000 * 3);
            break;
        default:
            break;
        }
    }

    if (key_status == KEY_EVENT_CLICK) {
        switch (key_number) {
        case 0:
            /* show all provisioned devices */
            Provisioner_show_cdb_node();
            break;
        case 1:
            /* Clearing provisioned node records */
            clear_store_node();
            break;
        case 4:
            /* Initiate provisioning for the last scanned device uuid */
            bt_mesh_provision_adv(unprov_uuid, PROV_NET_IDX, BT_MESH_ADDR_UNASSIGNED, 0);
            break;
        default:
            break;
        }
    }
}

static void mod_app_bind(void)
{
    int err;

    err = bt_mesh_cfg_cli_mod_app_bind(PROV_NET_IDX, PROVISIONER_ADDR, PROVISIONER_ADDR, PROV_APP_IDX,
                                       BT_MESH_MODEL_ID_GEN_ONOFF_CLI, NULL);
    if (err) {
        log_error("Failed to bind app-key (err %d)\n", err);
        return;
    }

    log_info("Configuration complete\n");

    struct bt_mesh_cdb_node *node;
    uint16_t addr = 0x01;
    do {
        node = bt_mesh_cdb_node_get(addr);
        if (node) {
            log_info("cdb node addr: 0x%x , onoff cli send 10 times.", node->addr);
            for (int i = 0; i < 10; i++) {
                onoff_cli_publish(node->addr);
                os_time_dly(100);
            }
            addr++;
        } else {
            log_info("provisioner don't have node now.");
            break;
        }
    } while (1);

}

/**
 * @brief Provisioner config self.
 *
 */
static void configure_self(void)
{
    int err;

    log_info("Configuring self...\n");

    /* Add Application Key */
    err = bt_mesh_cfg_cli_app_key_add(PROV_NET_IDX, PROVISIONER_ADDR, PROV_NET_IDX, PROV_APP_IDX,
                                      prov_app_key, NULL);

    if (err) {
        log_error("Failed to add app-key (err %d)\n", err);
        return;
    }

    /* Bind to onoff client model should be tasked until app key add*/
    sys_timeout_add(NULL, mod_app_bind, 100);
}

static void mesh_init(void)
{
    log_info("--func=%s", __FUNCTION__);

    int err = bt_mesh_init(&prov, &composition);
    if (err) {
        log_error("Initializing mesh failed (err %d)\n", err);
        return;
    }

    if (IS_ENABLED(CONFIG_BT_SETTINGS)) {
        settings_load();
    }

    bt_mesh_cdb_create(prov_net_key);

    err = bt_mesh_provision(prov_net_key, PROV_NET_IDX, flags, PROV_IV_IDX, PROVISIONER_ADDR, prov_dev_key);

    configure_self();
}

void bt_ble_init(void)
{
    u8 bt_addr[6] = {MAC_TO_LITTLE_ENDIAN(CUR_DEVICE_MAC_ADDR)};

    bt_mac_addr_set(bt_addr);

    mesh_setup(mesh_init);
}

#endif /* (CONFIG_MESH_MODEL == SIG_MESH_PROVISIONER) */
