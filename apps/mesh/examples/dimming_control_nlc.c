/*  Bluetooth Mesh */

/*
 * Copyright (c) 2024 Zhuhai Jieli technology Co.,Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "btstack/bluetooth.h"
#include "system/includes.h"
#include "bt_common.h"
#include "api/sig_mesh_api.h"
#include "model_api.h"
#include "api/gen_lvl.h"
#include "api/gen_lvl_cli.h"
#include "model_types.h"
#include "feature_correct.h"

#define LOG_TAG             "[Mesh-DIC_NLC]"
#define LOG_ERROR_ENABLE
#define LOG_DEBUG_ENABLE
#define LOG_INFO_ENABLE
/* #define LOG_DUMP_ENABLE */
#define LOG_CLI_ENABLE
#include "debug.h"

#if (CONFIG_MESH_MODEL == SIG_MESH_DIMMING_CONTROL_NLC)

#define     DELTA_MOVE_STEP                 1000
#define     HEALTH_SRV_LED_PORT             IO_PORTA_01
#define     BLE_DEV_NAME                    'D', 'i', 'm', 'm', 'i', 'n', 'g', 'C', 't', 'r', 'l', 'N', 'l', 'c'

/**
 * @brief Config current node features(Relay/Proxy/Friend/Low Power)
 */
/*-----------------------------------------------------------*/
#define BT_MESH_FEAT_SUPPORTED_TEMP         ( \
                                                BT_MESH_FEAT_RELAY | \
                                                BT_MESH_FEAT_PROXY | \
                                                0 \
                                            )


/**
 * @brief Conifg MAC of current demo
 */
/*-----------------------------------------------------------*/
#define CUR_DEVICE_MAC_ADDR         0x112233445570
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
BT_MESH_MODEL_PUB_DEFINE(gen_level_pub_cli, NULL, 2 + 2);
BT_MESH_HEALTH_PUB_DEFINE(health_pub, 0);

/* Company Identifiers (see Bluetooth Assigned Numbers) */
#define BT_COMP_ID_LF                          0x05D6 // Zhuhai Jieli technology Co.,Ltd

/** Dimming Control NLC Profile 1.0 */
#define BT_MESH_NLC_PROFILE_ID_DIMMING_CONTROL 0x1603


/** Context for a single Light Dimmer instance. */
struct dimmer_ctx {
    /** Generic Level client instance for this switch */
    struct bt_mesh_lvl_cli lvl_client;
};

static void health_attention_on(struct bt_mesh_model *mod);
static void health_attention_off(struct bt_mesh_model *mod);

/** @cond INTERNAL_HIDDEN */
extern const struct bt_mesh_model_op _bt_mesh_lvl_cli_op[];
/** @endcond */
const int config_bt_mesh_features = BT_MESH_FEAT_SUPPORTED;

/**
 * @brief Conifg complete local name
 */
/*-----------------------------------------------------------*/

static u8 ble_mesh_adv_name[32 + 2];
static const u8_t dev_uuid[16] = {MAC_TO_LITTLE_ENDIAN(CUR_DEVICE_MAC_ADDR)};

const uint8_t mesh_default_name[] = {
    // Name
    BYTE_LEN(BLE_DEV_NAME) + 1, BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME, BLE_DEV_NAME,
};

static const struct bt_mesh_health_srv_cb health_srv_cb = {
    .attn_on = health_attention_on,
    .attn_off = health_attention_off,
};

static struct bt_mesh_health_srv health_srv = {
    .cb = &health_srv_cb,
};

#if IS_ENABLED(CONFIG_BT_MESH_NLC_PERF_CONF)
static const uint8_t cmp2_elem_offset[1] = { 0 };

static const struct bt_mesh_comp2_record comp_rec[1] = {
    {
        .id = BT_MESH_NLC_PROFILE_ID_DIMMING_CONTROL,
        .version.x = 1,
        .version.y = 0,
        .version.z = 0,
        .elem_offset_cnt = 1,
        .elem_offset = cmp2_elem_offset,
        .data_len = 0
    },
};

static const struct bt_mesh_comp2 comp_p2 = {
    .record_cnt = 1,
    .record = comp_rec
};
#endif

/*
 *
 * Element Model Declarations
 *
 * Element 0 Root Models
 */
struct bt_mesh_model root_models[] = {
    BT_MESH_MODEL_CFG_SRV,
    BT_MESH_MODEL_HEALTH_SRV(&health_srv, &health_pub),
    BT_MESH_MODEL(BT_MESH_MODEL_ID_GEN_LEVEL_CLI, _bt_mesh_lvl_cli_op, &gen_level_pub_cli, NULL),
};

static struct bt_mesh_model *mod_cli_level[] = {
    &root_models[2],
};

static struct bt_mesh_elem elements[] = {
    BT_MESH_ELEM(0, root_models, BT_MESH_MODEL_NONE),
};

static const struct bt_mesh_comp comp = {
    .cid = BT_COMP_ID_LF,
    .elem = elements,
    .elem_count = ARRAY_SIZE(elements),
};

void get_mesh_adv_name(u8 *len, u8 **data)
{
    *len = ble_mesh_adv_name[0] + 1;
    *data = ble_mesh_adv_name;
}

static void health_attention_on(struct bt_mesh_model *mod)
{
    log_info(">>>>>>>>>>>>>>>>>health_attention_on\n");
    led_blink_worker_on(HEALTH_SRV_LED_PORT);
}

static void health_attention_off(struct bt_mesh_model *mod)
{
    log_info(">>>>>>>>>>>>>>>>>health_attention_off\n");
    led_blink_worker_off(HEALTH_SRV_LED_PORT);
}

static const struct bt_mesh_comp *composition_init(void)
{
#if IS_ENABLED(CONFIG_BT_MESH_NLC_PERF_CONF)
    if (bt_mesh_comp2_register(&comp_p2)) {
        log_error("Failed to register comp2\n");
    }
#endif
    return &comp;
}

static void provis_reset(void)
{
    bt_mesh_prov_enable(BT_MESH_PROV_GATT);
}

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
    .reset = provis_reset,
};

static void level_publish_move_set(u16_t move_set)
{
    struct dimmer_ctx ctx = {
        .lvl_client.model = mod_cli_level[0],
    };
    struct bt_mesh_lvl_move_set _move_set = {
        .delta = move_set,
        .transition = &(struct bt_mesh_model_transition){ .time = 500 },
    };
    /* As we can't know how many nodes are in a group, it doesn't
     * make sense to send acknowledged messages to group addresses.
     */
    if (bt_mesh_model_pub_is_unicast(ctx.lvl_client.model)) {
        bt_mesh_lvl_cli_move_set(&ctx.lvl_client, NULL, &_move_set, NULL);
    } else {
        bt_mesh_lvl_cli_move_set_unack(&ctx.lvl_client, NULL, &_move_set);
    }
}

static void mesh_init(void)
{
    log_info("--func=%s", __FUNCTION__);
    bt_conn_cb_register(bt_conn_get_callbacks());

    int err = bt_mesh_init(&prov, composition_init());
    if (err) {
        log_error("Initializing mesh failed (err %d)\n", err);
        return;
    }

    if (IS_ENABLED(CONFIG_BT_SETTINGS)) {
        settings_load();
    }

    bt_mesh_prov_enable(BT_MESH_PROV_GATT);
}

void input_key_handler(u8 key_status, u8 key_number)
{
    log_info("key_number=0x%x", key_number);

    if ((key_number == 0) && (key_status == KEY_EVENT_CLICK)) {
        log_info("\n  Start dimming  UP \n");
        level_publish_move_set(DELTA_MOVE_STEP);
        return;
    }

    if ((key_number == 1) && (key_status == KEY_EVENT_CLICK)) {
        log_info("\n  Start dimming  DOWN \n");
        level_publish_move_set(-DELTA_MOVE_STEP);
        return;
    }

    if ((key_number == 2) && (key_status == KEY_EVENT_LONG)) {
        log_info("\n  <bt_mesh_reset> \n");
        bt_mesh_reset();
        return;
    }
}

void bt_ble_init(void)
{
    u8 bt_addr[6] = {MAC_TO_LITTLE_ENDIAN(CUR_DEVICE_MAC_ADDR)};

    bt_mac_addr_set(bt_addr);

    u8 *name_p = &ble_mesh_adv_name[2];
    int ret = syscfg_read(CFG_BT_NAME, name_p, 32);
    if (ret <= 0) {
        log_info("read bt name err\n");
        memcpy(ble_mesh_adv_name, mesh_default_name, sizeof(mesh_default_name));
    } else {
        ble_mesh_adv_name[0] = strlen(name_p) + 1;
        ble_mesh_adv_name[1] = BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME;
        put_buf(name_p, 32);
    }

    mesh_set_gap_name(name_p);
    log_info("mesh_name:%s\n", name_p);

    mesh_setup(mesh_init);
    if (BT_MODE_IS(BT_BQB)) {
        ble_bqb_test_thread_init();
    }
}
#endif /* (CONFIG_MESH_MODEL == SIG_MESH_DIMMING_CONTROL_NLC) */
