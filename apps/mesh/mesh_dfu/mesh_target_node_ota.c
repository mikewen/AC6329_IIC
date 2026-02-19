/*************************************************************************************************/
/*!
*  \file       mesh_target_node_ota.c
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

#include "system/app_core.h"
#include "system/malloc.h"
#include "mesh_target_node_ota.h"
#include "system/includes.h"
#include "model_api.h"

#define LOG_TAG             "[Mesh_node_ota]"
#define LOG_INFO_ENABLE
//#define LOG_DEBUG_ENABLE
#define LOG_ERROR_ENABLE
/* #define LOG_DUMP_ENABLE */
#define LOG_CLI_ENABLE
#include "mesh_log.h"

#if (CONFIG_MESH_MODEL == SIG_MESH_DFU_TARGET_DEMO)

/**************************************************************************************************
Macros
**************************************************************************************************/
/* OTA每笔写入flash的数据大小 */
#define OTA_WRITE_FLASH_SIZE       (4096)


/**************************************************************************************************
Data Types
**************************************************************************************************/
typedef struct {
    volatile uint32_t rx_len;
    uint32_t file_size;
    uint32_t version;
    uint16_t ota_packet_num;
    volatile uint8_t ota_state;
    uint8_t res_byte;
    uint8_t  buff[OTA_WRITE_FLASH_SIZE + 16];
} mesh_targe_ota_t;


/**************************************************************************************************
Global Variables
**************************************************************************************************/
static mesh_targe_ota_t *mesh_targe_ota;
static u32 mesh_target_ota_total_rev_data_len;


/**************************************************************************************************
Functions Declarations
**************************************************************************************************/
static void mesh_targe_ota_set_state(uint8_t state);


/**************************************************************************************************
Functions
**************************************************************************************************/
/*************************************************************************************************/
/*!
 *  \brief      ota 模块初始化
 *
 *  \return     ota 模块初始化结果
 *
 */
/*************************************************************************************************/
static int mesh_targe_ota_init(void)
{
    log_debug("%s", __FUNCTION__);

    if (mesh_targe_ota) {
        mesh_targe_ota_exit();
    }

    if (!mesh_targe_ota) {
        mesh_targe_ota = malloc(sizeof(mesh_targe_ota_t));
    }

    if (mesh_targe_ota) {
        memset(mesh_targe_ota, 0, sizeof(mesh_targe_ota_t));
        set_ota_status(1);
        return MESH_TARGET_OTA_OP_SUCC;
    }

    log_error("%s malloc fail", __FUNCTION__);

    return MESH_TARGET_OTA_OP_MALLOC_FAIL;
}

/*************************************************************************************************/
/*!
 *  \brief  ota模块退出
 *
 *  \return ota模块状态
 *
 */
/*************************************************************************************************/
static int mesh_targe_ota_exit(void)
{
    log_debug("%s", __FUNCTION__);

    dual_bank_passive_update_exit(NULL);
    set_ota_status(0);

    if (mesh_targe_ota) {
        local_irq_disable();
        free(mesh_targe_ota);
        mesh_targe_ota = NULL;
        local_irq_enable();
    }

    mesh_targe_ota_set_state(MESH_TARGET_OTA_STATE_IDLE);

    return MESH_TARGET_OTA_OP_SUCC;
}

/*************************************************************************************************/
/*!
 *  \brief      ota状态更新
 *
 *  \param      [in] state
 *
 */
/*************************************************************************************************/
static void mesh_targe_ota_set_state(uint8_t state)
{
    if (mesh_targe_ota != NULL) {
        mesh_targe_ota->ota_state = state;
    }
}

/*************************************************************************************************/
/*!
 *  \brief      获取ota状态
 *
 *  \param      [in]
 *
 *  \return     MESH_TARGET_OTA_st_e
 *
 *  \note
 */
/*************************************************************************************************/
static MESH_TARGET_OTA_st_e mesh_targe_ota_get_state(void)
{
    if (mesh_targe_ota != NULL) {
        return mesh_targe_ota->ota_state;
    }

    return MESH_TARGET_OTA_STATE_IDLE;
}

/*************************************************************************************************/
/*!
 *  \brief      ota 完成 重启系统
 *
 *  \param      [in]
 *
 *  \note
 */
/*************************************************************************************************/
static void mesh_targe_ota_reset(void *priv)
{
    log_info("cpu_reset!");
    cpu_reset();
}

/*************************************************************************************************/
/*!
 *  \brief      升级启动文件检测回调
 *
 *  \param      [in]
 *
 *  \return     0 升级成功；-1 升级失败
 *
 *  \note
 */
/*************************************************************************************************/
static int mesh_targe_ota_boot_info_cb(int err)
{
    log_debug("mesh_targe_ota_boot_info_cb:%d", err);
    if (err == 0) {
        // sys_timeout_add(NULL, mesh_targe_ota_reset, 1000);
    } else {
        log_error("update head fail");
        return -1;
    }

    return 0;
}

/*************************************************************************************************/
/*!
 *  \brief      升级启动文件校验回调
 *
 *  \param      [in]
 *
 *  \return     0 升级成功；-1 升级失败
 *
 *  \note
 */
/*************************************************************************************************/
static int verify_ota_end(int res)
{
    if (res) {
        log_debug("ota verify success");
        mesh_targe_ota_set_state(MESH_TARGET_OTA_STATE_IDLE);
        dual_bank_update_burn_boot_info(mesh_targe_ota_boot_info_cb);
        return 0;
    } else {
        log_error("ota verify fail");
        return -1;
    }
}

/*************************************************************************************************/
/*!
 *  \brief      传输文件数据写入结束
 *
 *  \param      [in]
 *
 *  \return
 *
 *  \note
 */
/*************************************************************************************************/
static int mesh_targe_ota_write_complete(void)
{
    dual_bank_update_verify(NULL, NULL, verify_ota_end);
    set_ota_status(0);

    return MESH_TARGET_OTA_OP_SUCC;
}

/*************************************************************************************************/
/*!
 *  \brief      每一笔数据写入flash的回调
 *
 *  \param      [in] err id
 *
 *  \return
 *
 *  \note
 */
/*************************************************************************************************/
static int mesh_targe_update_write_cb(int err)
{
    log_debug("flash write %u bytes Complete, err %d", mesh_targe_ota->rx_len, err);

    if (err) {
        mesh_targe_ota_set_state(MESH_TARGET_OTA_STATE_WRITE_ERROR);
        log_error("OTA write failed err %d", err);
        return MESH_TARGET_OTA_OP_WRITE_FAIL;
    }

    mesh_targe_ota->rx_len = 0;

    return MESH_TARGET_OTA_OP_SUCC;
}

/*************************************************************************************************/
/*!
 *  \brief      ota升级主要消息流程处理
 *
 *  \param      [in] cmd_type, recv_data,recv_len
 *
 *  \return
 *
 *  \note
 */
/*************************************************************************************************/
int mesh_targe_ota_process(dfu_node_cmd_type_t cmd_type, uint8_t *recv_data, uint32_t recv_len, uint32_t reverse_data, uint32_t offset)
{
    int ret = 0; /* must init to 0 */
    int status = MESH_TARGET_OTA_OP_SUCC;

    log_debug("cmd_type= %d,len= %u", cmd_type, recv_len);

    /* 检测OTA初始化是否处理 */
    // if (!(mesh_targe_ota != NULL)) {
    //     log_error("OTA not running!");
    //     return  MESH_TARGET_OTA_OP_INIT_FAIL;
    // }

    switch (cmd_type) {
    case DFU_NODE_OTA_REQ: {
        /* 请求升级命令 */
        status = mesh_targe_ota_init();
        if (status == MESH_TARGET_OTA_OP_SUCC) {
            mesh_targe_ota_set_state(MESH_TARGET_OTA_STATE_REQ);
            mesh_targe_ota->version = little_endian_read_32(recv_data, 0);
            log_debug("OTA_REQ,version= %08x", mesh_targe_ota->version);
        }
        break;
    }

    case DFU_NODE_OTA_FILE_INFO: {
        /* 升级文件信息下发，检测空间 */
        mesh_targe_ota_set_state(MESH_TARGET_OTA_STATE_CHECK_FILESIZE);
        mesh_targe_ota->file_size = recv_len;

        log_debug("OTA_FILE_INFO file_size= %u reverse_data = 0x%x", mesh_targe_ota->file_size, reverse_data);

        ret = dual_bank_passive_update_init(reverse_data, mesh_targe_ota->file_size, OTA_WRITE_FLASH_SIZE, NULL);
        if (0 == ret) {
            /* 检测空间是否不足 */
            ret = dual_bank_update_allow_check(mesh_targe_ota->file_size);
            if (ret) {
                log_error("check err: %d", ret);
                status = MESH_TARGET_OTA_OP_NO_SPACE;
            }
        } else {
            log_error("init err: %d", ret);
            status = MESH_TARGET_OTA_OP_INIT_FAIL;
        }
        break;
    }

    case DFU_NODE_OTA_DATA: {
        //每一笔升级文件数据写入
        if (mesh_targe_ota_get_state() == MESH_TARGET_OTA_STATE_WRITE_ERROR) {
            ret = MESH_TARGET_OTA_OP_WRITE_FAIL;
            goto write_end;
        }

        int wait_cnt = 200;//wait flash write complete, timeout is 2 second
        while (wait_cnt && mesh_targe_ota->rx_len >= OTA_WRITE_FLASH_SIZE) {
            putchar('&');
            wait_cnt--;
            os_time_dly(1);
        }

        if (mesh_targe_ota->rx_len && wait_cnt == 0) {
            ret = MESH_TARGET_OTA_OP_OTHER_ERR;
            goto write_end;
        }

        mesh_targe_ota_set_state(MESH_TARGET_OTA_STATE_WRITE_DATA);
        mesh_targe_ota->ota_packet_num++;
        memcpy(&mesh_targe_ota->buff[offset], recv_data, recv_len); /* copy to 4K backup buffer*/
        mesh_targe_ota->rx_len += recv_len;
        mesh_target_ota_total_rev_data_len += recv_len;
        log_debug("OTA_DATA: offset= %d rx_len= %d", offset, mesh_targe_ota->rx_len);

        if (mesh_targe_ota->rx_len == reverse_data) {
            log_debug("WRITE: reverse_data= %d",  reverse_data);
            ret = dual_bank_update_write(mesh_targe_ota->buff, reverse_data, mesh_targe_update_write_cb);
        }

write_end:
        if (ret) {
            log_error("dual_write err %d", ret);
            status = MESH_TARGET_OTA_OP_WRITE_FAIL;
            /* mesh_targe_ota_boot_info_cb(0);//to reset */
        }
    }
    break;

    case DFU_NODE_OTA_END:
        /* 传输文件结束 */
        log_debug("OTA_END");
        mesh_targe_ota_set_state(MESH_TARGET_OTA_STATE_COMPLETE);
        status = mesh_targe_ota_write_complete();
        break;

    case DFU_NODE_OTA_FAIL:
        /* 链路断开 */
        log_debug("OTA_FAIL");
        if (mesh_targe_ota_get_state() != MESH_TARGET_OTA_STATE_IDLE) {
            log_error("OTA fail");
            /* mesh_targe_ota_boot_info_cb(0); //to reset */
        }
        status = mesh_targe_ota_exit();
        break;

    default:
        log_error("unknow target ota cmd");
        break;

    }

    if (status) {
        log_error("ota_process err= %d", status);
        mesh_targe_ota_exit();
    }
    return status;
}

#endif

