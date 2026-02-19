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
#include "access.h"
#include "api/lightness_srv.h"
#include "api/light_ctrl_srv.h"
#include "feature_correct.h"

#define LOG_TAG "[Mesh-BLS_NLC]"
#define LOG_ERROR_ENABLE
#define LOG_DEBUG_ENABLE
#define LOG_INFO_ENABLE
/* #define LOG_DUMP_ENABLE */
#define LOG_CLI_ENABLE
#include "debug.h"

#if (CONFIG_MESH_MODEL == SIG_MESH_BASIC_LIGHTNESS_CTRL_NLC)

#define LED_LVL_PIN             IO_PORTA_01
#define PWM_SIZE_STEP           512
/**
 * @brief Config current node features(Relay/Proxy/Friend/Low Power)
 */
/*-----------------------------------------------------------*/
#define BT_MESH_FEAT_SUPPORTED_TEMP ( \
    BT_MESH_FEAT_RELAY |              \
    BT_MESH_FEAT_PROXY |              \
    0)

#define BLE_DEV_NAME 'B', 'a', 's', 'i', 'c', 'L', 'i', 'g', 'h', 't', 'n', 'e', 's', 's', 'C', 't', 'r', 'l', 'N', 'l', 'c'
/**
 * @brief Conifg MAC of current demo
 */
/*-----------------------------------------------------------*/
#define CUR_DEVICE_MAC_ADDR         0x112233445571

/* Company Identifiers (see Bluetooth Assigned Numbers) */
#define BT_COMP_ID_LF               0x05D6 // Zhuhai Jieli technology Co.,Ltd

/** Basic Lightness Controller NLC Profile 1.0 */
#define BT_MESH_NLC_PROFILE_ID_BASIC_LIGHTNESS_CONTROLLER 0x1601

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

struct lightness_ctx {
    struct bt_mesh_lightness_srv lightness_srv;
    uint16_t target_lvl;
    uint16_t current_lvl;
    uint32_t rem_time;
    uint32_t time_per;
};
//-------------------------------------------------------------------
static void provis_reset(void);
static void health_attention_on(struct bt_mesh_model *mod);
static void health_attention_off(struct bt_mesh_model *mod);
static void start_new_light_trans(const struct bt_mesh_lightness_set *set,
                                  struct lightness_ctx *ctx);
static void light_set(struct bt_mesh_lightness_srv *srv,
                      struct bt_mesh_msg_ctx *ctx,
                      const struct bt_mesh_lightness_set *set,
                      struct bt_mesh_lightness_status *rsp);
static void light_get(struct bt_mesh_lightness_srv *srv,
                      struct bt_mesh_msg_ctx *ctx,
                      struct bt_mesh_lightness_status *rsp);
//-------------------------------------------------------------------
const int config_bt_mesh_features = BT_MESH_FEAT_SUPPORTED;

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

#if IS_ENABLED(CONFIG_BT_MESH_NLC_PERF_CONF)
static const uint8_t cmp2_elem_offset[2] = {0, 1};

static const struct bt_mesh_comp2_record comp_rec[1] = {
    {
        .id = BT_MESH_NLC_PROFILE_ID_BASIC_LIGHTNESS_CONTROLLER,
        .version.x = 1,
        .version.y = 0,
        .version.z = 0,
        .elem_offset_cnt = 2,
        .elem_offset = cmp2_elem_offset,
        .data_len = 0
    },
};

static const struct bt_mesh_comp2 comp_p2 = {
    .record_cnt = 1,
    .record = comp_rec
};
#endif

static const u8_t dev_uuid[16] = {MAC_TO_LITTLE_ENDIAN(CUR_DEVICE_MAC_ADDR)};
//-------------------------------------------------------------------

static const struct bt_mesh_health_srv_cb health_srv_cb = {
    .attn_on = health_attention_on,
    .attn_off = health_attention_off,
};

static struct bt_mesh_health_srv health_srv = {
    .cb = &health_srv_cb,
};
//-------------------------------------------------------------
void get_mesh_adv_name(u8 *len, u8 **data)
{
    // r_printf("==============================%s,%d\n", __FUNCTION__, __LINE__);
    // put_buf(ble_mesh_adv_name,32);
    *len = ble_mesh_adv_name[0] + 1;
    *data = ble_mesh_adv_name;
}

static void set_onoff_led_duty(u16 duty)
{
    log_info(">>>>>set_onoff_led_duty = %d\n", duty);
#if defined(CONFIG_CPU_BD19)
    set_timer_pwm_duty(JL_TIMER1, duty);
#elif defined(CONFIG_CPU_BR25)
    set_timer_pwm_duty(JL_TIMER5, duty);
#endif
}

static void health_attention_on(struct bt_mesh_model *mod)
{
    log_info(">>>>>>>>>>>>>>>>>health_attention_on\n");
#if defined(CONFIG_CPU_BD19)
    set_timer_pwm_duty(JL_TIMER1, 10000);
#elif defined(CONFIG_CPU_BR25)
    set_timer_pwm_duty(JL_TIMER5, 10000);
#endif
}

static void health_attention_off(struct bt_mesh_model *mod)
{
    log_info(">>>>>>>>>>>>>>>>>health_attention_off\n");
#if defined(CONFIG_CPU_BD19)
    set_timer_pwm_duty(JL_TIMER1, 0);
#elif defined(CONFIG_CPU_BR25)
    set_timer_pwm_duty(JL_TIMER5, 0);
#endif

}

static void led_worker(void *priv)
{
    struct lightness_ctx *l_ctx = (struct lightness_ctx *)priv;
    l_ctx->rem_time -= l_ctx->time_per;
    log_info("l_ctx->target_lvl = %d, l_ctx->current_lvl = %d, l_ctx->rem_time = %d, l_ctx->time_per = %d\n",
             l_ctx->target_lvl, l_ctx->current_lvl, l_ctx->rem_time, l_ctx->time_per);
    if ((l_ctx->rem_time <= l_ctx->time_per) ||
        (abs(l_ctx->target_lvl - l_ctx->current_lvl) <= PWM_SIZE_STEP)) {
        struct bt_mesh_lightness_status status = {
            .current = l_ctx->target_lvl,
            .target = l_ctx->target_lvl,
        };

        l_ctx->current_lvl = l_ctx->target_lvl;
        l_ctx->rem_time = 0;

        bt_mesh_lightness_srv_pub(&l_ctx->lightness_srv, NULL, &status);

        goto apply_and_print;
    } else if (l_ctx->target_lvl > l_ctx->current_lvl) {
        l_ctx->current_lvl += PWM_SIZE_STEP;
    } else {
        l_ctx->current_lvl -= PWM_SIZE_STEP;
    }

apply_and_print:
    uint16_t clamped_lvl = bt_mesh_lightness_clamp(&l_ctx->lightness_srv,
                           l_ctx->current_lvl);
    set_onoff_led_duty(clamped_lvl);
    log_info("Current light lvl: %u/65535\n", clamped_lvl);
}

static void start_new_light_trans(const struct bt_mesh_lightness_set *set,
                                  struct lightness_ctx *ctx)
{
    uint32_t step_cnt = abs(set->lvl - ctx->current_lvl) / PWM_SIZE_STEP;
    uint32_t time = set->transition ? set->transition->time : 0;
    uint32_t delay = set->transition ? set->transition->delay : 0;

    ctx->target_lvl = set->lvl;
    ctx->time_per = (step_cnt ? time / step_cnt : 0);
    ctx->rem_time = time;
    sys_timeout_add((void *)ctx, led_worker, K_MSEC(delay) + 10);
    log_info("New light transition-> Lvl: %d, Time: %d, Delay: %d\n",
             set->lvl, time, delay);
}

static void light_set(struct bt_mesh_lightness_srv *srv,
                      struct bt_mesh_msg_ctx *ctx,
                      const struct bt_mesh_lightness_set *set,
                      struct bt_mesh_lightness_status *rsp)
{
    struct lightness_ctx *l_ctx =
        CONTAINER_OF(srv, struct lightness_ctx, lightness_srv);

    start_new_light_trans(set, l_ctx);
    rsp->current = l_ctx->rem_time ? l_ctx->current_lvl : l_ctx->target_lvl;
    rsp->target = l_ctx->target_lvl;
    rsp->remaining_time = set->transition ? set->transition->time : 0;
}

static void light_get(struct bt_mesh_lightness_srv *srv,
                      struct bt_mesh_msg_ctx *ctx,
                      struct bt_mesh_lightness_status *rsp)
{
    struct lightness_ctx *l_ctx =
        CONTAINER_OF(srv, struct lightness_ctx, lightness_srv);

    rsp->current = bt_mesh_lightness_clamp(&l_ctx->lightness_srv, l_ctx->current_lvl);
    rsp->target = l_ctx->target_lvl;
    rsp->remaining_time = l_ctx->rem_time;
}

static const struct bt_mesh_lightness_srv_handlers lightness_srv_handlers = {
    .light_set = light_set,
    .light_get = light_get,
};

static struct lightness_ctx my_ctx = {
    .lightness_srv = BT_MESH_LIGHTNESS_SRV_INIT(&lightness_srv_handlers),
};

static struct bt_mesh_scene_srv scene_srv;

static struct bt_mesh_light_ctrl_srv light_ctrl_srv =
    BT_MESH_LIGHT_CTRL_SRV_INIT(&my_ctx.lightness_srv);


static struct bt_mesh_elem elements[] = {
    BT_MESH_ELEM(0, BT_MESH_MODEL_LIST(
        BT_MESH_MODEL_CFG_SRV,
        BT_MESH_MODEL_HEALTH_SRV(&health_srv, &health_pub),
        BT_MESH_MODEL_LIGHTNESS_SRV(&my_ctx.lightness_srv),
        BT_MESH_MODEL_SCENE_SRV(&scene_srv)),
    BT_MESH_MODEL_NONE),
    BT_MESH_ELEM(1,
    BT_MESH_MODEL_LIST(
        BT_MESH_MODEL_LIGHT_CTRL_SRV(&light_ctrl_srv)),
    BT_MESH_MODEL_NONE),
};

static const struct bt_mesh_comp comp = {
    .cid = BT_COMP_ID_LF,
    .elem = elements,
    .elem_count = ARRAY_SIZE(elements),
};

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
    log_info("key_number=0x%x", key_number);

    if ((key_number == 2) && (key_status == KEY_EVENT_LONG)) {
        log_info("\n  <bt_mesh_reset> \n");
        bt_mesh_reset();
        return;
    }

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

    default:
        return;
    }
}

static void mesh_lvl_led_init(void)
{
#if defined(CONFIG_CPU_BD19)
    timer_pwm_init(JL_TIMER1, LED_LVL_PIN, 10000, 0);
#elif defined(CONFIG_CPU_BR25)
    timer_pwm_init(JL_TIMER5, 10000, 0, LED_LVL_PIN, 0);
#endif
}

void bt_ble_init(void)
{
    // init pwm led first.
    mesh_lvl_led_init();

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
#endif /* (CONFIG_MESH_MODEL == SIG_MESH_BASIC_LIGHTNESS_CTRL_NLC) */
