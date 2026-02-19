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
#include "api/sensor_srv.h"
#include "api/properties.h"
#include "feature_correct.h"
#include "access.h"
#include "api/sensor_types.h"

#define LOG_TAG "[Mesh-SENSOR_NLC]"
#define LOG_ERROR_ENABLE
#define LOG_DEBUG_ENABLE
#define LOG_INFO_ENABLE
/* #define LOG_DUMP_ENABLE */
#define LOG_CLI_ENABLE
#include "debug.h"

#if (CONFIG_MESH_MODEL == SIG_MESH_OCCUPACY_SENSOR_NLC)
/**
 * @brief Config current node features(Relay/Proxy/Friend/Low Power)
 */
/*-----------------------------------------------------------*/
#define BT_MESH_FEAT_SUPPORTED_TEMP ( \
    BT_MESH_FEAT_RELAY |              \
    BT_MESH_FEAT_PROXY |              \
    0)

#define BLE_DEV_NAME 'L', 'i', 'g', 'h', 't', 'O', 'c', 'c', 'p', 'a', 'c', 'y', 'S', 'e', 'n', 's', 'o', 'r', 'N', 'l', 'c'

/**
 * @brief Conifg MAC of current demo
 */
/*-----------------------------------------------------------*/
#define CUR_DEVICE_MAC_ADDR 0x762233445566
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

#define HEALTH_SRV_LED_PORT     IO_PORTA_01

/* Company Identifiers (see Bluetooth Assigned Numbers) */
#define BT_COMP_ID_LF 0x05D6 // Zhuhai Jieli technology Co.,Ltd

/** Occupancy Sensor NLC Profile 1.0 */
#define BT_MESH_NLC_PROFILE_ID_OCCUPANCY_SENSOR 0x1605

static double amb_light_level_ref;
static double amb_light_level_gain = 1.0;
/* Using a dummy ambient light value because we do not have a real ambient light sensor. */
static double dummy_ambient_light_value = 1.0;

static struct sensor_value pres_mot_thres = {.val1 = 10};

static int32_t pres_detect;
static uint32_t prev_detect;
static uint32_t presence_detected_timer;


static void provis_reset(void);
static int time_since_presence_detected_get(struct bt_mesh_sensor_srv *srv,
        struct bt_mesh_sensor *sensor,
        struct bt_mesh_msg_ctx *ctx,
        struct sensor_value *rsp);

static void presence_motion_threshold_get(struct bt_mesh_sensor_srv *srv,
        struct bt_mesh_sensor *sensor,
        const struct bt_mesh_sensor_setting *setting,
        struct bt_mesh_msg_ctx *ctx,
        struct sensor_value *rsp);

static int presence_motion_threshold_set(struct bt_mesh_sensor_srv *srv,
        struct bt_mesh_sensor *sensor,
        const struct bt_mesh_sensor_setting *setting,
        struct bt_mesh_msg_ctx *ctx,
        const struct sensor_value *value);
static int presence_detected_get(struct bt_mesh_sensor_srv *srv,
                                 struct bt_mesh_sensor *sensor,
                                 struct bt_mesh_msg_ctx *ctx,
                                 struct sensor_value *rsp);

static void health_attention_off(struct bt_mesh_model *mod);
static void health_attention_on(struct bt_mesh_model *mod);
//----------------------------------------------------------------
const int config_bt_mesh_features = BT_MESH_FEAT_SUPPORTED;

#if IS_ENABLED(CONFIG_BT_MESH_NLC_PERF_CONF)
static const uint8_t cmp2_elem_offset1[1] = {0};

static const struct bt_mesh_comp2_record comp_rec[1] = {
    {
        .id = BT_MESH_NLC_PROFILE_ID_OCCUPANCY_SENSOR,
        .version.x = 1,
        .version.y = 0,
        .version.z = 0,
        .elem_offset_cnt = 1,
        .elem_offset = cmp2_elem_offset1,
        .data_len = 0
    }
};

static const struct bt_mesh_comp2 comp_p2 = {
    .record_cnt = 1,
    .record = comp_rec
};
#endif

static struct bt_mesh_sensor time_since_presence_detected = {
    .type = &bt_mesh_sensor_time_since_presence_detected,
    .get = time_since_presence_detected_get,
};

static struct bt_mesh_sensor_setting presence_motion_threshold_setting[] = {{
        .type = &bt_mesh_sensor_motion_threshold,
        .get = presence_motion_threshold_get,
        .set = presence_motion_threshold_set,
    }
};

static struct bt_mesh_sensor presence_sensor = {
    .type = &bt_mesh_sensor_presence_detected,
    .get = presence_detected_get,
    .settings = {
        .list = (const struct bt_mesh_sensor_setting *) &presence_motion_threshold_setting,
        .count = ARRAY_SIZE(presence_motion_threshold_setting),
    },
};

static struct bt_mesh_sensor *const occupancy_sensor[] = {
    &presence_sensor,
    &time_since_presence_detected,
};

static const struct bt_mesh_health_srv_cb health_srv_cb = {
    .attn_on = health_attention_on,
    .attn_off = health_attention_off,
};

static struct bt_mesh_health_srv health_srv = {
    .cb = &health_srv_cb,
};

static struct bt_mesh_sensor_srv occupancy_sensor_srv =
    BT_MESH_SENSOR_SRV_INIT(occupancy_sensor, ARRAY_SIZE(occupancy_sensor));

static struct bt_mesh_elem elements[] = {
    BT_MESH_ELEM(0, BT_MESH_MODEL_LIST(BT_MESH_MODEL_CFG_SRV, BT_MESH_MODEL_HEALTH_SRV(&health_srv, &health_pub), BT_MESH_MODEL_SENSOR_SRV(&occupancy_sensor_srv)),
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


static int presence_detected_get(struct bt_mesh_sensor_srv *srv,
                                 struct bt_mesh_sensor *sensor,
                                 struct bt_mesh_msg_ctx *ctx,
                                 struct sensor_value *rsp)
{
    rsp->val1 = pres_detect;
    return 0;
};

static int time_since_presence_detected_get(struct bt_mesh_sensor_srv *srv,
        struct bt_mesh_sensor *sensor,
        struct bt_mesh_msg_ctx *ctx,
        struct sensor_value *rsp)
{
    if (pres_detect) {
        rsp->val1 = 0;
    } else if (prev_detect) {
        rsp->val1 = MIN((k_uptime_get_32() - prev_detect) / MSEC_PER_SEC, 0xFFFF);
    } else {
        /* Before first detection, the time since last detection is unknown. Returning
         * unknown value until a detection is done. Value is defined in specification.
         */
        rsp->val1 = 0xFFFF;
    }

    return 0;
}

static void presence_motion_threshold_get(struct bt_mesh_sensor_srv *srv,
        struct bt_mesh_sensor *sensor,
        const struct bt_mesh_sensor_setting *setting,
        struct bt_mesh_msg_ctx *ctx,
        struct sensor_value *rsp)
{
    rsp[0] = pres_mot_thres;
    log_info("Presence motion threshold: %u [%d ms]\n", rsp[0].val1, 100 * rsp[0].val1);
}

static int presence_motion_threshold_set(struct bt_mesh_sensor_srv *srv,
        struct bt_mesh_sensor *sensor,
        const struct bt_mesh_sensor_setting *setting,
        struct bt_mesh_msg_ctx *ctx,
        const struct sensor_value *value)
{
    pres_mot_thres = value[0];
    log_info("Presence motion threshold: %u [%d ms]\n", value[0].val1, 100 * value[0].val1);

    if (IS_ENABLED(CONFIG_BT_SETTINGS)) {
        int err;

        err = settings_save_one("presence/motion_threshold",
                                &pres_mot_thres, sizeof(pres_mot_thres));
        if (err) {
            log_info("Error storing setting (%d)\n", err);
        } else {
            log_info("Stored setting\n");
        }
    }
    return 0;
}

static void presence_detected()
{
    int err;

    /* This sensor value must be boolean -
     * .val1 can only be '0' or '1'
     */
    struct sensor_value val = {
        .val1 = 1,
    };

    err = bt_mesh_sensor_srv_pub(&occupancy_sensor_srv, NULL, &presence_sensor, &val);

    if (err) {
        log_error("Error publishing end of presence (%d)\n", err);
    }

    pres_detect = 1;
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

    if ((key_number == 1) && (key_status == KEY_EVENT_CLICK)) {
        if (pres_mot_thres.val1 && !presence_detected_timer) {
            presence_detected_timer = sys_timer_add(NULL, presence_detected, 100 * pres_mot_thres.val1);
        }
    }

    if ((key_number == 3) && (key_status == KEY_EVENT_CLICK)) {
        if (presence_detected_timer) {
            sys_timer_del(presence_detected_timer);
            presence_detected_timer = 0x0;

            int err;

            /* This sensor value must be boolean -
             * .val1 can only be '0' or '1'
             */
            struct sensor_value val = {
                .val1 = 0,
            };

            err = bt_mesh_sensor_srv_pub(&occupancy_sensor_srv, NULL,
                                         &presence_sensor, &val);

            if (err) {
                log_info("Error publishing presence (%d)\n", err);
            }

            pres_detect = 0;
            prev_detect = k_uptime_get_32();
        }
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

#endif /* CONFIG_MESH_MODEL == SIG_MESH_OCCUPACY_SENSOR_NLC */

