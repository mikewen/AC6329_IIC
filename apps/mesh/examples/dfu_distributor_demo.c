/*************************************************************************************************/
/*!
*  \file       dfu_distributor_demo.c
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
#include "dfd_srv_internal.h"
#include "feature_correct.h"
#include "dfu_dist.h"

#define LOG_TAG             "[Mesh-DFU_Dist_demo]"
#define LOG_ERROR_ENABLE
#define LOG_DEBUG_ENABLE
#define LOG_INFO_ENABLE
#define LOG_DUMP_ENABLE
//#define LOG_CLI_ENABLE
#include "debug.h"

#if (CONFIG_MESH_MODEL == SIG_MESH_DFU_DISTRIBUTOR_DEMO)

/**************************************************************************************************
Macros
**************************************************************************************************/
/* Config current node features(Relay/Proxy/Friend/Low Power) */
#define BT_MESH_FEAT_SUPPORTED_TEMP         ( \
                                                BT_MESH_FEAT_RELAY | \
                                                BT_MESH_FEAT_PROXY | \
                                                0 \
                                            )

#define BLE_DEV_NAME        'D', 'F', 'U', '_', 'd', 'i', 's', 't', '_', 'd', 'e', 'm', 'o'

/* Conifg MAC of current demo */
#define CUR_DEVICE_MAC_ADDR         0xbb2233445567

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

/* Company Identifiers (see Bluetooth Assigned Numbers) */
#define BT_COMP_ID_LF           0x05D6 // Zhuhai Jieli technology Co.,Ltd


/**************************************************************************************************
Function declaration
**************************************************************************************************/
static void health_attention_on(struct bt_mesh_model *mod);
static void health_attention_off(struct bt_mesh_model *mod);


/**************************************************************************************************
Global Variables
**************************************************************************************************/
static struct bt_mesh_blob_io_flash blob_flash_stream;

const int config_bt_mesh_features = BT_MESH_FEAT_SUPPORTED;

/* Conifg complete local name */
static u8 ble_mesh_adv_name[32 + 2];

const uint8_t mesh_default_name[] = {
    BYTE_LEN(BLE_DEV_NAME) + 1, BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME, BLE_DEV_NAME,
};

static const struct bt_mesh_health_srv_cb health_srv_cb = {
    .attn_on = health_attention_on,
    .attn_off = health_attention_off,
};

static struct bt_mesh_health_srv health_srv = {
    .cb = &health_srv_cb,
};

/* Element 0 Root Models */
struct bt_mesh_model root_models[] = {
    BT_MESH_MODEL_CFG_SRV,
    BT_MESH_MODEL_HEALTH_SRV(&health_srv, &health_pub),
    BT_MESH_MODEL_DFD_SRV(&dfd_srv),
};

static struct bt_mesh_elem elements[] = {
    BT_MESH_ELEM(1, root_models, BT_MESH_MODEL_NONE),
};

static const struct bt_mesh_comp comp = {
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

static const struct bt_mesh_model *mod;


/**************************************************************************************************
Functions
**************************************************************************************************/
void get_mesh_adv_name(u8 *len, u8 **data)
{
    //r_printf("==============================%s,%d\n", __FUNCTION__, __LINE__);
    //put_buf(ble_mesh_adv_name,32);

    *len = ble_mesh_adv_name[0] + 1;
    *data = ble_mesh_adv_name;
}

/*************************************************************************************************/
/*!
*  \brief  used for health model.
*/
/*************************************************************************************************/
static void health_attention_on(struct bt_mesh_model *mod)
{
    log_info(">>>>>>>>>>>>>>>>>health_attention_on\n");
}

static void health_attention_off(struct bt_mesh_model *mod)
{
    log_info(">>>>>>>>>>>>>>>>>health_attention_off\n");
}

static void mesh_init(void)
{
    log_info("--func=%s", __FUNCTION__);

    int err;

    bt_conn_cb_register(bt_conn_get_callbacks());

    err = bt_mesh_init(&prov, &comp);
    if (err) {
        log_error("Initializing mesh failed (err %d)\n", err);
        return;
    }

    if (IS_ENABLED(CONFIG_BT_SETTINGS)) {
        settings_load();
    }

    bt_mesh_prov_enable(BT_MESH_PROV_ADV | BT_MESH_PROV_GATT);
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
        if (key_number == 0) {
            log_info("  [KEY_EVENT_CLICK]");
        }
        if (key_number == 1) {
            log_info("  [KEY_EVENT_CLICK]");
        }
        if (key_number == 3) {
            log_info("  [KEY_EVENT_CLICK]");
        }
        if (key_number == 4) {
            log_info("  [KEY_EVENT_CLICK]");
        }
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
}

extern void mesh_set_gap_name(const u8 *name);
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

    int err;
    err = bt_mesh_blob_io_flash_init(&blob_flash_stream, 0, 0);
    if (err) {
        log_error("Failed to init BLOB IO Flash module: %d\n", err);
    }

    dfu_distributor_init(&blob_flash_stream);

    mesh_setup(mesh_init);

    if (BT_MODE_IS(BT_BQB)) {
        ble_bqb_test_thread_init();
    }
}
#endif /* (CONFIG_MESH_MODEL == SIG_MESH_DFU_DISTRIBUTOR_DEMO) */
