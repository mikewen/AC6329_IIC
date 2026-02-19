/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "btstack/bluetooth.h"
#include "system/includes.h"
#include "bt_common.h"
#include "api/sig_mesh_api.h"
#include "model_types.h"
#include "adaptation.h"

#define LOG_TAG             "[Gen_model_utils]"
#define LOG_ERROR_ENABLE
#define LOG_DEBUG_ENABLE
#define LOG_INFO_ENABLE
/* #define LOG_DUMP_ENABLE */
#define LOG_CLI_ENABLE
#include "debug.h"

#define CONFIG_BT_MESH_MOD_ACKD_TIMEOUT_BASE 3000
#define CONFIG_BT_MESH_MOD_ACKD_TIMEOUT_PER_HOP 50

/** Unknown encoded transition time value */
#define TRANSITION_TIME_UNKNOWN (0x3F)

/** Delay field step factor in milliseconds */
#define DELAY_TIME_STEP_MS (5)

#define SYS_FOREVER_MS (-1)

static const u32_t model_transition_res[] = {
    100,
    1 * MSEC_PER_SEC,
    10 * MSEC_PER_SEC,
    60 * 10 * MSEC_PER_SEC,
};

s32_t model_transition_decode(u8_t encoded_transition)
{
    u8_t steps = (encoded_transition & 0x3f);
    u8_t res = (encoded_transition >> 6);

    return (steps == TRANSITION_TIME_UNKNOWN) ?
           SYS_FOREVER_MS :
           (model_transition_res[res] * steps);
}

u8_t model_transition_encode(s32_t transition_time)
{
    if (transition_time == 0) {
        return 0;
    }
    if (transition_time == SYS_FOREVER_MS) {
        return TRANSITION_TIME_UNKNOWN;
    }

    /* Even though we're only able to encode values up to 6200 ms in
     * 100 ms step resolution, values between 6200 and 6600 ms are closer
     * to 6200 ms than 7000 ms, which would be the nearest 1-second step
     * representation. Similar cases exist for the other step resolutions
     * too, so we'll use these custom limits instead of checking against
     * the max value of each step resolution. The formula for the limit is
     * ((0x3e * res[i]) + (((0x3e * res[i]) / res[i+1]) + 1) * res[i+1])/2.
     */
    const s32_t limits[] = {
        6600,
        66 * MSEC_PER_SEC,
        910 * MSEC_PER_SEC,
        60 * 620 * MSEC_PER_SEC,
    };

    for (u8_t i = 0; i < ARRAY_SIZE(model_transition_res); ++i) {
        if (transition_time > limits[i]) {
            continue;
        }

        u8_t steps = ((transition_time + model_transition_res[i] / 2) /
                      model_transition_res[i]);
        steps = MAX(1, steps);
        steps = MIN(0x3e, steps);
        return (i << 6) | steps;
    }

    return TRANSITION_TIME_UNKNOWN;
}


u8_t model_delay_encode(u32_t delay)
{
    return delay / DELAY_TIME_STEP_MS;
}

s32_t model_delay_decode(u8_t encoded_delay)
{
    return encoded_delay * DELAY_TIME_STEP_MS;
}

static inline void
model_transition_buf_add(struct net_buf_simple *buf,
                         const struct bt_mesh_model_transition *transition)
{
    net_buf_simple_add_u8(buf, model_transition_encode(transition->time));
    net_buf_simple_add_u8(buf, model_delay_encode(transition->delay));
}

s32_t model_ackd_timeout_get(const struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx)
{
    u8_t ttl = (ctx ? ctx->send_ttl : model->pub->ttl);
    s32_t time = (CONFIG_BT_MESH_MOD_ACKD_TIMEOUT_BASE +
                  ttl * CONFIG_BT_MESH_MOD_ACKD_TIMEOUT_PER_HOP);

    return time;
}

bool bt_mesh_model_pub_is_unicast(const struct bt_mesh_model *model)
{
    return model->pub && BT_MESH_ADDR_IS_UNICAST(model->pub->addr);
}

int tid_check_and_update(struct bt_mesh_tid_ctx *prev_transaction, uint8_t tid,
                         const struct bt_mesh_msg_ctx *ctx)
{
    /* This updates the timestamp in the progress: */
    int64_t uptime_delta = k_uptime_delta(&prev_transaction->timestamp);

    if (prev_transaction->src == ctx->addr &&
        prev_transaction->dst == ctx->recv_dst &&
        prev_transaction->tid == tid && uptime_delta < 6000) {
        return -EALREADY;
    }

    prev_transaction->src = ctx->addr;
    prev_transaction->dst = ctx->recv_dst;
    prev_transaction->tid = tid;

    return 0;
}

