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

#if (CONFIG_MESH_MODEL == SIG_MESH_AMBIENT_LIGHT_SENSOR_NLC)
/**
 * @brief Config current node features(Relay/Proxy/Friend/Low Power)
 */
/*-----------------------------------------------------------*/
#define BT_MESH_FEAT_SUPPORTED_TEMP ( \
    BT_MESH_FEAT_RELAY |              \
    BT_MESH_FEAT_PROXY |              \
    0)
#define BLE_DEV_NAME   'A', 'm', 'b', 'i', 'e', 'n', 't', 'L', 'i', 'g', 'h', 't', 'S', 'e', 'n', 's', 'o', 'r', 'N', 'l', 'c'

/**
 * @brief Conifg MAC of current demo
 */
/*-----------------------------------------------------------*/
#define CUR_DEVICE_MAC_ADDR             0x112233445575
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

#define HEALTH_SRV_LED_PORT IO_PORTA_01

/* Company Identifiers (see Bluetooth Assigned Numbers) */
#define BT_COMP_ID_LF 0x05D6 // Zhuhai Jieli technology Co.,Ltd

/** Ambient Light Sensor NLC Profile 1.0 */
#define BT_MESH_NLC_PROFILE_ID_AMBIENT_LIGHT_SENSOR 0x1600

static double amb_light_level_ref;
static double amb_light_level_gain = 1.0;
/* Using a dummy ambient light value because we do not have a real ambient light sensor. */
static double dummy_ambient_light_value = 1.0;

static struct sensor_value pres_mot_thres = {.val1 = 10};

static int32_t pres_detect;
static uint32_t prev_detect;
static uint32_t presence_detected_timer;

static void provis_reset(void);
static int amb_light_level_get(struct bt_mesh_sensor_srv *srv,
                               struct bt_mesh_sensor *sensor,
                               struct bt_mesh_msg_ctx *ctx,
                               struct sensor_value *rsp);
static void amb_light_level_gain_get(struct bt_mesh_sensor_srv *srv,
                                     struct bt_mesh_sensor *sensor,
                                     const struct bt_mesh_sensor_setting *setting,
                                     struct bt_mesh_msg_ctx *ctx,
                                     struct sensor_value *rsp);
static int amb_light_level_gain_set(struct bt_mesh_sensor_srv *srv,
                                    struct bt_mesh_sensor *sensor,
                                    const struct bt_mesh_sensor_setting *setting,
                                    struct bt_mesh_msg_ctx *ctx,
                                    const struct sensor_value *value);
static void amb_light_level_ref_get(struct bt_mesh_sensor_srv *srv,
                                    struct bt_mesh_sensor *sensor,
                                    const struct bt_mesh_sensor_setting *setting,
                                    struct bt_mesh_msg_ctx *ctx,
                                    struct sensor_value *rsp);

static int amb_light_level_ref_set(struct bt_mesh_sensor_srv *srv,
                                   struct bt_mesh_sensor *sensor,
                                   const struct bt_mesh_sensor_setting *setting,
                                   struct bt_mesh_msg_ctx *ctx,
                                   const struct sensor_value *value);

static void health_attention_off(struct bt_mesh_model *mod);
static void health_attention_on(struct bt_mesh_model *mod);
//----------------------------------------------------------------
const int config_bt_mesh_features = BT_MESH_FEAT_SUPPORTED;

static const double dummy_amb_light_values[] = {
    0.01,
    100.00,
    200.00,
    500.00,
    750.00,
    1000.00,
    10000.00,
    167772.13,
};

#if IS_ENABLED(CONFIG_BT_MESH_NLC_PERF_CONF)
static const uint8_t cmp2_elem_offset1[1] = {0};

static const struct bt_mesh_comp2_record comp_rec[1] = {
    {
        .id = BT_MESH_NLC_PROFILE_ID_AMBIENT_LIGHT_SENSOR,
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

// ambient light info
static struct bt_mesh_sensor_column amb_light_level_range = {
    .start = {.val1 = 0, .val2 = 0},
    .end = {.val1 = 167772, .val2 = 130000},
};

static const struct bt_mesh_sensor_descriptor present_amb_light_desc = {
    .tolerance = {
        .negative = {
            .val1 = 0,
        },
        .positive = {
            .val1 = 0,
        }
    },
    .sampling_type = BT_MESH_SENSOR_SAMPLING_UNSPECIFIED,
    .period = 0,
    .update_interval = 0,
};

static struct bt_mesh_sensor_setting amb_light_level_setting[] = {
    {
        .type = &bt_mesh_sensor_present_amb_light_level,
        .get = amb_light_level_ref_get,
        .set = amb_light_level_ref_set,
    },
    {
        .type = &bt_mesh_sensor_gain,
        .get = amb_light_level_gain_get,
        .set = amb_light_level_gain_set,
    },
};

static struct bt_mesh_sensor present_amb_light_level = {
    .type = &bt_mesh_sensor_present_amb_light_level,
    .get = amb_light_level_get,
    .descriptor = &present_amb_light_desc,
    .settings = {
        .list = (const struct bt_mesh_sensor_setting *) &amb_light_level_setting,
        .count = ARRAY_SIZE(amb_light_level_setting),
    },
};

static struct bt_mesh_sensor *const ambient_light_sensor[] = {
    &present_amb_light_level,
};

static const struct bt_mesh_health_srv_cb health_srv_cb = {
    .attn_on = health_attention_on,
    .attn_off = health_attention_off,
};

static struct bt_mesh_health_srv health_srv = {
    .cb = &health_srv_cb,
};

static struct bt_mesh_sensor_srv ambient_light_sensor_srv =
    BT_MESH_SENSOR_SRV_INIT(ambient_light_sensor, ARRAY_SIZE(ambient_light_sensor));

static struct bt_mesh_elem elements[] = {
    BT_MESH_ELEM(0, BT_MESH_MODEL_LIST(BT_MESH_MODEL_CFG_SRV, BT_MESH_MODEL_HEALTH_SRV(&health_srv, &health_pub), BT_MESH_MODEL_SENSOR_SRV(&ambient_light_sensor_srv)),
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

/**
 * @brief Helper function for converting double to struct sensor_value.
 *
 * @param val A pointer to a sensor_value struct.
 * @param inp The converted value.
 * @return 0 if successful, negative errno code if failure.
 */
static inline int sensor_value_from_double(struct sensor_value *val, double inp)
{
    if (inp < INT32_MIN || inp > INT32_MAX) {
        return -ERANGE;
    }

    double val2 = (inp - (int32_t)inp) * 1000000.0;

    if (val2 < INT32_MIN || val2 > INT32_MAX) {
        return -ERANGE;
    }

    val->val1 = (int32_t)inp;
    val->val2 = (int32_t)val2;

    return 0;
}

/**
 * @brief Helper function for converting struct sensor_value to double.
 *
 * @param val A pointer to a sensor_value struct.
 * @return The converted value.
 */
static inline double sensor_value_to_double(const struct sensor_value *val)
{
    return (double)val->val1 + (double)val->val2 / 1000000;
}

static void amb_light_level_gain_get(struct bt_mesh_sensor_srv *srv,
                                     struct bt_mesh_sensor *sensor,
                                     const struct bt_mesh_sensor_setting *setting,
                                     struct bt_mesh_msg_ctx *ctx,
                                     struct sensor_value *rsp)
{
    (void)sensor_value_from_double(rsp, amb_light_level_gain);
    log_info("Ambient light level gain: %s\n", bt_mesh_sensor_ch_str(rsp));
};

static void amb_light_level_gain_store(double gain)
{
    amb_light_level_gain = gain;

    if (IS_ENABLED(CONFIG_BT_SETTINGS)) {
        int err;

        err = settings_save_one("amb_light_level/gain",
                                &amb_light_level_gain, sizeof(amb_light_level_gain));
        if (err) {
            log_info("Error storing setting (%d)\n", err);
        } else {
            log_info("Stored setting\n");
        }
    }
}

static int amb_light_level_gain_set(struct bt_mesh_sensor_srv *srv,
                                    struct bt_mesh_sensor *sensor,
                                    const struct bt_mesh_sensor_setting *setting,
                                    struct bt_mesh_msg_ctx *ctx,
                                    const struct sensor_value *value)
{
    log_info("value->val1 = %d, value->val2 = %d\n", value->val1, value->val2);
    amb_light_level_gain_store(sensor_value_to_double(value));
    log_info("Ambient light level gain: %s\n", bt_mesh_sensor_ch_str(value));

    return 0;
}

static int amb_light_level_get(struct bt_mesh_sensor_srv *srv,
                               struct bt_mesh_sensor *sensor,
                               struct bt_mesh_msg_ctx *ctx,
                               struct sensor_value *rsp)
{
    int err;

    /* Report ambient light as dummy value, and changing it by pressing
     * a button. The logic and hardware for measuring the actual ambient
     * light usage of the device should be implemented here.
     */
    double reported_value = amb_light_level_gain * dummy_ambient_light_value;

    err = sensor_value_from_double(rsp, reported_value);
    if (err) {
        log_info("Error getting ambient light level sensor data (%d)\n", err);
    }

    if (!bt_mesh_sensor_value_in_column(rsp, &amb_light_level_range)) {
        if (amb_light_level_range.start.val1 == amb_light_level_range.end.val1) {
            if (rsp->val2 <= amb_light_level_range.start.val2) {
                *rsp = amb_light_level_range.start;
            } else {
                *rsp = amb_light_level_range.end;
            }
        } else if (rsp->val1 <= amb_light_level_range.start.val1) {
            *rsp = amb_light_level_range.start;
        } else {
            *rsp = amb_light_level_range.end;
        }
    }

    return err;
}

static int amb_light_level_ref_set(struct bt_mesh_sensor_srv *srv,
                                   struct bt_mesh_sensor *sensor,
                                   const struct bt_mesh_sensor_setting *setting,
                                   struct bt_mesh_msg_ctx *ctx,
                                   const struct sensor_value *value)
{
    struct sensor_value gain_value_tmp;

    amb_light_level_ref = sensor_value_to_double(value);

    /* When using the a real ambient light sensor the sensor value should be
     * read and used instead of the dummy value.
     */
    if (dummy_ambient_light_value > 0.0) {
        amb_light_level_gain_store(amb_light_level_ref / dummy_ambient_light_value);
    } else {
        amb_light_level_gain_store(99999);
    }

    sensor_value_from_double(&gain_value_tmp, amb_light_level_gain);

    log_info("Ambient light level ref(%s) ", bt_mesh_sensor_ch_str(value));
    log_info("gain(%s)\n", bt_mesh_sensor_ch_str(&gain_value_tmp));

    return 0;
}

static void amb_light_level_ref_get(struct bt_mesh_sensor_srv *srv,
                                    struct bt_mesh_sensor *sensor,
                                    const struct bt_mesh_sensor_setting *setting,
                                    struct bt_mesh_msg_ctx *ctx,
                                    struct sensor_value *rsp)
{
    (void)sensor_value_from_double(rsp, amb_light_level_ref);
    log_info("Ambient light level ref: %s\n", bt_mesh_sensor_ch_str(rsp));
};

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

    settings_load();

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

    if ((key_number == 0) && (key_status == KEY_EVENT_CLICK)) {
        int err;
        static int amb_light_idx;
        struct sensor_value val;

        dummy_ambient_light_value = dummy_amb_light_values[amb_light_idx++];
        amb_light_idx = amb_light_idx % ARRAY_SIZE(dummy_amb_light_values);

        err = sensor_value_from_double(&val, dummy_ambient_light_value);
        if (err) {
            log_error("Error getting ambient light level sensor data (%d)\n", err);
        }

        log_info("Ambient light level: %s\n", bt_mesh_sensor_ch_str(&val));
        err = bt_mesh_sensor_srv_pub(&ambient_light_sensor_srv, NULL,
                                     &present_amb_light_level, &val);
        if (err) {
            log_error("Error publishing present ambient light level (%d)\n", err);
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

#endif /* (CONFIG_MESH_MODEL == SIG_MESH_AMBIENT_LIGHT_SENSOR_NLC) */
