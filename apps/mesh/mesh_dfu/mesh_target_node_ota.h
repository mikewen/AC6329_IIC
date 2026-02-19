/*************************************************************************************************/
/*!
*  \file       mesh_target_node_ota.h
*
*  \brief      BLE baseband interface file.
*
*  Copyright (c) 2011-2024 ZhuHai Jieli Technology Co.,Ltd.
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

#ifndef MESH_TARGET_NODE_OTA_H
#define MESH_TARGET_NODE_OTA_H

#ifdef __cplusplus
extern "C" {
#endif

/**************************************************************************************************
Constants
**************************************************************************************************/
/*! \brief      mesh target dfu flash write state. */
typedef enum {
    MESH_TARGET_OTA_STATE_IDLE = 0,
    MESH_TARGET_OTA_STATE_REQ,
    MESH_TARGET_OTA_STATE_CHECK_FILESIZE,
    MESH_TARGET_OTA_STATE_WRITE_DATA,
    MESH_TARGET_OTA_STATE_COMPLETE,
    MESH_TARGET_OTA_STATE_WRITE_ERROR,
} MESH_TARGET_OTA_st_e;

/*! \brief      mesh target dfu flash write cmd. */
typedef enum {
    DFU_NODE_OTA_REQ = 1,
    DFU_NODE_OTA_FILE_INFO,
    DFU_NODE_OTA_DATA,
    DFU_NODE_OTA_END,
    DFU_NODE_OTA_FAIL,
} dfu_node_cmd_type_t;

/*! \brief      mesh target dfu operation state. */
enum {
    MESH_TARGET_OTA_OP_SUCC = 0,
    MESH_TARGET_OTA_OP_NO_SPACE,
    MESH_TARGET_OTA_OP_INIT_FAIL,
    MESH_TARGET_OTA_OP_WRITE_FAIL,
    MESH_TARGET_OTA_OP_CRC_FAIL,
    MESH_TARGET_OTA_OP_MALLOC_FAIL,
    MESH_TARGET_OTA_OP_PKT_NUM_ERR,
    MESH_TARGET_OTA_OP_LEN_ERR,
    MESH_TARGET_OTA_OP_OTHER_ERR,
};

#ifdef __cplusplus
};
#endif

#endif /* MESH_TARGET_NODE_OTA_H */
