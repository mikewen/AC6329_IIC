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
#include "api/sensor_cli.h"
#include "api/properties.h"
#include "feature_correct.h"
#include "access.h"

#define LOG_TAG "[Mesh-sensor_cli_NLC]"
#define LOG_ERROR_ENABLE
#define LOG_DEBUG_ENABLE
#define LOG_INFO_ENABLE
/* #define LOG_DUMP_ENABLE */
#define LOG_CLI_ENABLE
#include "debug.h"

#if (CONFIG_MESH_MODEL == SIG_MESH_SENSOR_OBSEVER_NLC)
/**
 * @brief Config current node features(Relay/Proxy/Friend/Low Power)
 */
/*-----------------------------------------------------------*/
#define BT_MESH_FEAT_SUPPORTED_TEMP ( \
    BT_MESH_FEAT_RELAY |              \
    BT_MESH_FEAT_PROXY |              \
    0)

#define BLE_DEV_NAME                'L', 'i', 'g', 'h', 't', 'O', 'c', 'c', 'p', 'a', 'c', 'y', 'O', 'b', 's', 'e', 'r', 'v', 'e', 'r'

/**
 * @brief Conifg MAC of current demo
 */
/*-----------------------------------------------------------*/
#define CUR_DEVICE_MAC_ADDR         0x992233445567
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
BT_MESH_HEALTH_PUB_DEFINE(health_pub, 0);

#define         HEALTH_SRV_LED_PORT             IO_PORTA_01

/* Company Identifiers (see Bluetooth Assigned Numbers) */
#define BT_COMP_ID_LF 0x05D6 // Zhuhai Jieli technology Co.,Ltd

static struct sensor_value pres_mot_thres;

static int32_t pres_detect;
static uint32_t prev_detect;
static uint32_t presence_detected_timer;

static void provis_reset(void);
static void health_attention_off(struct bt_mesh_model *mod);
static void health_attention_on(struct bt_mesh_model *mod);
static void sensor_cli_desc_cb(struct bt_mesh_sensor_cli *cli, struct bt_mesh_msg_ctx *ctx,
                               const struct bt_mesh_sensor_info *sensor);
static void sensor_cli_data_cb(struct bt_mesh_sensor_cli *cli,
                               struct bt_mesh_msg_ctx *ctx,
                               const struct bt_mesh_sensor_type *sensor,
                               const struct sensor_value *value);
static void sensor_cli_setting_status_cb(struct bt_mesh_sensor_cli *cli,
        struct bt_mesh_msg_ctx *ctx,
        const struct bt_mesh_sensor_type *sensor,
        const struct bt_mesh_sensor_setting_status *setting);
static void sensor_cli_series_entry_cb(
    struct bt_mesh_sensor_cli *cli, struct bt_mesh_msg_ctx *ctx,
    const struct bt_mesh_sensor_type *sensor, uint8_t index, uint8_t count,
    const struct bt_mesh_sensor_series_entry *entry);

//----------------------------------------------------------------
const int config_bt_mesh_features = BT_MESH_FEAT_SUPPORTED;

static const struct bt_mesh_health_srv_cb health_srv_cb = {
    .attn_on = health_attention_on,
    .attn_off = health_attention_off,
};

static struct bt_mesh_health_srv health_srv = {
    .cb = &health_srv_cb,
};


static const struct bt_mesh_sensor_cli_handlers bt_mesh_sensor_cli_handlers = {
    .data = sensor_cli_data_cb,
    .series_entry = sensor_cli_series_entry_cb,
    .setting_status = sensor_cli_setting_status_cb,
    .sensor = sensor_cli_desc_cb,
};

static struct bt_mesh_sensor_cli sensor_cli =
    BT_MESH_SENSOR_CLI_INIT(&bt_mesh_sensor_cli_handlers);

static struct bt_mesh_elem elements[] = {
    BT_MESH_ELEM(0, BT_MESH_MODEL_LIST(BT_MESH_MODEL_CFG_SRV, BT_MESH_MODEL_HEALTH_SRV(&health_srv, &health_pub), BT_MESH_MODEL_SENSOR_CLI(&sensor_cli)),
    BT_MESH_MODEL_NONE)
};

static const struct bt_mesh_comp comp = {
    .cid = BT_COMP_ID_LF,
    .elem = elements,
    .elem_count = ARRAY_SIZE(elements),
};

/**
 * @brief Conifg complete local name
 */
/*-----------------------------------------------------------*/
static u8 ble_mesh_adv_name[32 + 2];

const uint8_t mesh_default_name[] = {
    // Name
    BYTE_LEN(BLE_DEV_NAME) + 1,
    BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME,
    BLE_DEV_NAME,
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
    .reset = provis_reset,
};
//--------------------------------------------------------------

static void sensor_cli_data_cb(struct bt_mesh_sensor_cli *cli,
                               struct bt_mesh_msg_ctx *ctx,
                               const struct bt_mesh_sensor_type *sensor,
                               const struct sensor_value *value)
{
    log_info(">>>>>>>>>>Sensor data received\n");
    log_info("sensor id: 0x%04x\n", sensor->id);
    if (sensor->id == BT_MESH_PROP_ID_PRESENT_AMB_LIGHT_LEVEL) {
        log_info("Ambient light level: %s\n", bt_mesh_sensor_ch_str(value));
    } else if (sensor->id == BT_MESH_PROP_ID_PRESENCE_DETECTED) {
        if (value->val1) {
            log_info("Presence detected\n");
        } else {
            log_info("No presence detected\n");
        }
    } else if (sensor->id == BT_MESH_PROP_ID_TIME_SINCE_PRESENCE_DETECTED) {
        if (!value->val1) {
            log_info("Presence detected, or under 1 second since presence detected\n");
        } else if (value->val1 == 0xFFFF) {
            log_info("Unknown last presence detected\n");
        } else {
            log_info("%s second(s) since last presence detected\n",
                     bt_mesh_sensor_ch_str(value));
        }
    }
}

static void sensor_cli_series_entry_cb(
    struct bt_mesh_sensor_cli *cli, struct bt_mesh_msg_ctx *ctx,
    const struct bt_mesh_sensor_type *sensor, uint8_t index, uint8_t count,
    const struct bt_mesh_sensor_series_entry *entry)
{
    log_info("Relative runtime in %d to %d degrees: %s percent\n",
             entry->value[1].val1, entry->value[2].val1,
             bt_mesh_sensor_ch_str(&entry->value[0]));
}

static void sensor_cli_setting_status_cb(struct bt_mesh_sensor_cli *cli,
        struct bt_mesh_msg_ctx *ctx,
        const struct bt_mesh_sensor_type *sensor,
        const struct bt_mesh_sensor_setting_status *setting)
{
    log_info("Sensor ID: 0x%04x, Setting ID: 0x%04x\n", sensor->id, setting->type->id);
    for (int chan = 0; chan < setting->type->channel_count; chan++) {
        log_info("\tChannel %d value: %s\n", chan,
                 bt_mesh_sensor_ch_str(&(setting->value[chan])));
    }
}

static void sensor_cli_desc_cb(struct bt_mesh_sensor_cli *cli, struct bt_mesh_msg_ctx *ctx,
                               const struct bt_mesh_sensor_info *sensor)
{
    log_info("Descriptor of sensor with ID 0x%04x:\n", sensor->id);
    log_info("\ttolerance: { positive: %s",
             bt_mesh_sensor_ch_str(&sensor->descriptor.tolerance.positive));
    log_info(" negative: %s }\n", bt_mesh_sensor_ch_str(&sensor->descriptor.tolerance.negative));
    log_info("\tsampling type: %d\n", sensor->descriptor.sampling_type);
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

void get_mesh_adv_name(u8 *len, u8 **data)
{
    // r_printf("==============================%s,%d\n", __FUNCTION__, __LINE__);
    // put_buf(ble_mesh_adv_name,32);

    *len = ble_mesh_adv_name[0] + 1;
    *data = ble_mesh_adv_name;
}

static const struct bt_mesh_comp *composition_init(void)
{
    return &comp;
}

static void provis_reset(void)
{
    bt_mesh_prov_enable(BT_MESH_PROV_GATT);
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
    if (!bt_mesh_is_provisioned()) {
        return;
    }

    log_info("key_number=0x%x", key_number);

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

#endif /* (CONFIG_MESH_MODEL == SIG_MESH_SENSOR_OBSEVER_NLC) */
