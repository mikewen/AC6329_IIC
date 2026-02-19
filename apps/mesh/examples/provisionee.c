/*  Bluetooth Mesh */

/*
 * Copyright (c) 2017 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "btstack/bluetooth.h"
#include "system/includes.h"
#include "bt_common.h"
#include "api/sig_mesh_api.h"
#include "model_api.h"
#include "provisioner_config.h"
#include "feature_correct.h"

#define LOG_TAG             "[Provisionee_demo]"
#define LOG_ERROR_ENABLE
#define LOG_DEBUG_ENABLE
#define LOG_INFO_ENABLE
/* #define LOG_DUMP_ENABLE */
#define LOG_CLI_ENABLE
#include "debug.h"

#if (CONFIG_MESH_MODEL == SIG_MESH_PROVISIONEE)

/**************************************************************************************************
Macros
**************************************************************************************************/
/* @brief Config current node features(Relay/Proxy/Friend/Low Power) */
#define BT_MESH_FEAT_SUPPORTED_TEMP         ( \
                                                0 \
                                            )

/* @brief Conifg complete local name */
#define BLE_DEV_NAME        'P', 'r', 'o', 'v', 'i', 's', 'i', 'o', 'n', 'e', 'e'

/* @brief Conifg MAC of current demo */
#define CUR_DEVICE_MAC_ADDR         0x112233445566

/* Model Operation Codes */
#define BT_MESH_MODEL_OP_GEN_ONOFF_GET			BT_MESH_MODEL_OP_2(0x82, 0x01)
#define BT_MESH_MODEL_OP_GEN_ONOFF_SET			BT_MESH_MODEL_OP_2(0x82, 0x02)
#define BT_MESH_MODEL_OP_GEN_ONOFF_SET_UNACK	BT_MESH_MODEL_OP_2(0x82, 0x03)
#define BT_MESH_MODEL_OP_GEN_ONOFF_STATUS		BT_MESH_MODEL_OP_2(0x82, 0x04)

/* Company Identifiers (see Bluetooth Assigned Numbers) */
#define BT_COMP_ID_LF           0x05D6 // Zhuhai Jieli technology Co.,Ltd

/* LED NUMBER */
#define LED0_GPIO_PIN           0

/**************************************************************************************************
Data Types
**************************************************************************************************/
struct onoff_state {
    u8_t current;
    u8_t previous;
    u8_t led_gpio_pin;
};

struct _switch {
    u8_t sw_num;
    u8_t onoff_state;
};

/**************************************************************************************************
Function declaration
**************************************************************************************************/
static void gen_onoff_get(struct bt_mesh_model *model,
                          struct bt_mesh_msg_ctx *ctx,
                          struct net_buf_simple *buf);

static void gen_onoff_set_unack(struct bt_mesh_model *model,
                                struct bt_mesh_msg_ctx *ctx,
                                struct net_buf_simple *buf);

static void gen_onoff_set(struct bt_mesh_model *model,
                          struct bt_mesh_msg_ctx *ctx,
                          struct net_buf_simple *buf);

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
BT_MESH_MODEL_PUB_DEFINE(gen_onoff_pub_srv, NULL, 2 + 2);

const int config_bt_mesh_features = BT_MESH_FEAT_SUPPORTED;

/* @brief Config proxy connectable adv hardware param */
#if BT_MESH_FEATURES_GET(BT_MESH_FEAT_LOW_POWER)
const u16 config_bt_mesh_proxy_node_adv_interval = ADV_SCAN_UNIT(3000); // unit: ms
#else
const u16 config_bt_mesh_proxy_node_adv_interval = ADV_SCAN_UNIT(300); // unit: ms
#endif /* BT_MESH_FEATURES_GET(BT_MESH_FEAT_LOW_POWER) */

const uint8_t mesh_name[] = {
    BYTE_LEN(BLE_DEV_NAME) + 1, BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME, BLE_DEV_NAME,
};

static struct bt_mesh_cfg_cli cfg_cli;

static u8_t trans_id;

static struct onoff_state onoff_state[] = {
    { .led_gpio_pin = LED0_GPIO_PIN },
};

const u8 led_use_port[1] = {

    IO_PORTA_01,

};

/* OnOff Model Server Op Dispatch Table */
static const struct bt_mesh_model_op gen_onoff_srv_op[] = {
    { BT_MESH_MODEL_OP_GEN_ONOFF_GET,       BT_MESH_LEN_EXACT(0), gen_onoff_get },
    { BT_MESH_MODEL_OP_GEN_ONOFF_SET,       BT_MESH_LEN_MIN(2), gen_onoff_set },
    { BT_MESH_MODEL_OP_GEN_ONOFF_SET_UNACK, BT_MESH_LEN_MIN(2), gen_onoff_set_unack },
    BT_MESH_MODEL_OP_END,
};

/*
 *
 * Element Model Declarations
 *
 * Element 0 Root Models
 */
static struct bt_mesh_model root_models[] = {
    BT_MESH_MODEL_CFG_SRV,
    BT_MESH_MODEL(BT_MESH_MODEL_ID_GEN_ONOFF_SRV, gen_onoff_srv_op, &gen_onoff_pub_srv, &onoff_state[0]),
};

/*
 * Button to Client Model Assignments
 */
static struct bt_mesh_model *mod_cli_sw[] = {
    &root_models[1],
};

/*
 * Root and Secondary Element Declarations
 */
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
};


/**************************************************************************************************
Functions
**************************************************************************************************/
void get_mesh_adv_name(u8 *len, u8 **data)
{
    *len = sizeof(mesh_name);

    *data = mesh_name;
}

static void gen_onoff_get(struct bt_mesh_model *model,
                          struct bt_mesh_msg_ctx *ctx,
                          struct net_buf_simple *buf)
{
    NET_BUF_SIMPLE_DEFINE(msg, 2 + 1 + 4);
    struct onoff_state *onoff_state = model->rt->user_data;

    log_info("addr 0x%04x onoff 0x%02x\n",
             bt_mesh_model_elem(model)->rt->addr, onoff_state->current);
    bt_mesh_model_msg_init(&msg, BT_MESH_MODEL_OP_GEN_ONOFF_STATUS);
    buffer_add_u8_at_tail(&msg, onoff_state->current);

    if (bt_mesh_model_send(model, ctx, &msg, NULL, (void *)ctx->recv_dst)) {
        log_info("Unable to send On Off Status response\n");
    }
}

static void gen_onoff_set_unack(struct bt_mesh_model *model,
                                struct bt_mesh_msg_ctx *ctx,
                                struct net_buf_simple *buf)
{
    struct net_buf_simple *msg = model->pub->msg;
    struct onoff_state *onoff_state = model->rt->user_data;
    int err;

    onoff_state->current = buffer_pull_u8_from_head(buf);
    log_info("addr 0x%02x state 0x%02x\n",
             bt_mesh_model_elem(model)->rt->addr, onoff_state->current);
    /* log_info_hexdump((u8 *)onoff_state, sizeof(*onoff_state)); */

    gpio_pin_write(onoff_state->led_gpio_pin,
                   onoff_state->current);
}

static void gen_onoff_set(struct bt_mesh_model *model,
                          struct bt_mesh_msg_ctx *ctx,
                          struct net_buf_simple *buf)
{
    log_info("gen_onoff_set\n");

    gen_onoff_set_unack(model, ctx, buf);
    gen_onoff_get(model, ctx, buf);
}

static void client_publish(struct _switch *sw)
{
    int err;
    struct bt_mesh_model *mod_cli;
    struct bt_mesh_model_pub *pub_cli;

    mod_cli = mod_cli_sw[sw->sw_num];
    pub_cli = mod_cli->pub;

    if (pub_cli->addr == BT_MESH_ADDR_UNASSIGNED) {
        return;
    }

    log_info("publish to 0x%04x onoff 0x%04x sw->sw_num 0x%04x\n",
             pub_cli->addr, sw->onoff_state, sw->sw_num);
    bt_mesh_model_msg_init(pub_cli->msg,
                           BT_MESH_MODEL_OP_GEN_ONOFF_SET);

    buffer_add_u8_at_tail(pub_cli->msg, sw->onoff_state);
    buffer_add_u8_at_tail(pub_cli->msg, trans_id++);
    err = bt_mesh_model_publish(mod_cli);
    if (err) {
        log_info("bt_mesh_model_publish err %d\n", err);
    }
}

/*
 * Button Pressed Worker Task
 */
static void button_pressed_worker(struct _switch *sw)
{
    if (sw->sw_num >= composition.elem_count) {
        log_info("sw_num over elem_count");
        return;
    }

    client_publish(sw);
}

static void mesh_init(void)
{
    log_info("--func=%s", __FUNCTION__);

    bt_conn_cb_register(bt_conn_get_callbacks());

    log_info(">>> uuid");
    printf_buf(prov.uuid, 16);

    int err = bt_mesh_init(&prov, &composition);
    if (err) {
        log_error("Initializing mesh failed (err %d)\n", err);
        return;
    }

    if (IS_ENABLED(CONFIG_BT_SETTINGS)) {
        settings_load();
    }

    bt_mesh_prov_enable(BT_MESH_PROV_GATT | BT_MESH_PROV_ADV);
}

void input_key_handler(u8 key_status, u8 key_number)
{
    struct _switch press_switch;

    log_info("key_number=0x%x", key_number);

    if ((key_number == 2) && (key_status == KEY_EVENT_CLICK)) {
        log_info("\n  <bt_mesh_reset> \n");
        bt_mesh_reset();
        return;
    }

    switch (key_status) {
    case KEY_EVENT_CLICK:
        log_info("  [KEY_EVENT_CLICK]  ");
        press_switch.sw_num = key_number;
        press_switch.onoff_state = 1;
        button_pressed_worker(&press_switch);
        break;
    case KEY_EVENT_LONG:
        log_info("  [KEY_EVENT_LONG]  ");
        press_switch.sw_num = key_number;
        press_switch.onoff_state = 0;
        button_pressed_worker(&press_switch);
        break;
    case KEY_EVENT_HOLD:
        log_info("  [KEY_EVENT_HOLD]  ");
        break;
    default :
        return;
    }
}

void bt_ble_init(void)
{
    u8 bt_addr[6] = {MAC_TO_LITTLE_ENDIAN(CUR_DEVICE_MAC_ADDR)};

    bt_mac_addr_set(bt_addr);

    mesh_setup(mesh_init);
}

#endif /* (CONFIG_MESH_MODEL == SIG_MESH_PROVISIONEE) */


