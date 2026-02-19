/*************************************************************************************************/
/*!
*  \file       mesh_dfu_app_cmd_protocol.h
*
*  \brief      distributor and app cmd protocol interface file.
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

#ifndef MESH_DFU_APP_CMD_PROTOCOL_H
#define MESH_DFU_APP_CMD_PROTOCOL_H

#include "cpu.h"

#ifdef __cplusplus
extern "C" {
#endif

/**************************************************************************************************
Macros
**************************************************************************************************/
#define ATT_CHARACTERISTIC_bd00_bd01_VALUE_HANDLE                0x0010
#define ATT_CHARACTERISTIC_bd00_bd02_VALUE_HANDLE                0x0012
#define ATT_CHARACTERISTIC_bd00_bd02_CLIENT_CONFIGURATION_HANDLE 0x0013


/**************************************************************************************************
Constants
**************************************************************************************************/
/*! \brief      mesh dfu app opcode type. */
typedef enum {
    MESH_DFU_APP_OP_FILE_TRANS = 0xA0,
    MESH_DFU_APP_OP_ADD_SLOT,
    MESH_DFU_APP_OP_ADD_TARGET,
    MESH_DFU_APP_OP_START,
    MESH_DFU_APP_OP_APPLY,
    MESH_DFU_APP_OP_CANCEL,
    MESH_DFU_APP_OP_DELETE_TARGET,
    MESH_DFU_APP_OP_READ_DFU_CFG,
    MESH_DFU_APP_OP_READ_DFU_STATE,
    MESH_DFU_APP_OP_NOTIFY_DFU_STATE,
} MESH_DFU_APP_OPCODE;

typedef enum {
    ACTION_TRANS_FILE = 0x01,
    ACTION_REQUEST_DATA,
    ACTION_PUSH_DATA,
    ACTION_STOP_TRANS,
    ACTION_CANCEL_TRANS,
} FILE_TRANS_ACTION;

typedef enum {
    OPERATION_SUCCESS = 0x00,
    OPERATION_FAILED,
    INVALID_PARAMETER,
    MEMORY_LACKED,
    UNDEFINED_COMMAND,
    DATA_OVERFLOW,
} RESPONSE_STATE;

typedef enum {
    ACTION_OPERATION_SUCCESS = 0x00,
    ACTION_MEMORY_LACKED,
    ACTION_WRITE_DATA_FAILED,
    ACTION_CRC_CHECK_FAILED,
    ACTION_WAIT_TIMEOUT,
} ACTION_RESULT_CODE;

typedef enum {
    ACTION_READ_CFG = 0x01,
    ACTION_TRANS_CFG,
    ACTION_STOP_TRANS_CFG,
} READ_DFU_CFG_ACTION;

typedef enum {
    OPERATION_READ_SUCCESS = 0x00,
    OPERATION_READ_STOP,
    READ_EXCEPTION,
} READ_DFU_STATE;

/**************************************************************************************************
Data Types
**************************************************************************************************/
typedef struct {
    uint8_t prev_sn;  // Previous Serial Number
    bool valid;             // Marks whether the opcode is valid (i.e., whether the packet was ever received)
} opcode_state_t;

struct file_parameter_t {
    uint8_t file_version[8];
    uint32_t file_size;
    uint16_t crc;
} __attribute__((packed));

struct response_action_filed_t {
    uint16_t length;        // data length（not include Length field）
    uint8_t flag;           // flag, cmd packet:0x81, response packet:0x00
    uint8_t op;             // opcode
    uint8_t sn;             // sequences number
    uint8_t state;
    uint8_t action;
    uint8_t response_len;
    uint8_t response;
} __attribute__((packed));

struct response_action_packet_t {
    uint16_t head_flag;     // header flag, fixed to 0x5A5A
    struct response_action_filed_t field;
    uint16_t crc;           // CRC check digit
} __attribute__((packed));


struct send_action_filed_t {
    uint16_t length;        // data length（not include Length field）
    uint8_t flag;           // flag, cmd packet:0x81, response packet:0x00
    uint8_t op;             // opcode
    uint8_t sn;             // sequences number
    uint8_t action;
    uint8_t parameter_len;
    uint32_t parameter;
} __attribute__((packed));

struct send_action_packet_t {
    uint16_t head_flag;     // header flag, fixed to 0x5A5A
    struct send_action_filed_t field;
    uint16_t crc;           // CRC check digit
} __attribute__((packed));

struct stop_trans_action_filed_t {
    uint16_t length;        // data length（not include Length field）
    uint8_t flag;           // flag, cmd packet:0x81, response packet:0x00
    uint8_t op;             // opcode
    uint8_t sn;             // sequences number
    uint8_t action;
    uint8_t parameter_len;
    uint8_t parameter;
} __attribute__((packed));

struct stop_trans_action_packet_t {
    uint16_t head_flag;     // header flag, fixed to 0x5A5A
    struct stop_trans_action_filed_t field;
    uint16_t crc;           // CRC check digit
} __attribute__((packed));

typedef struct {
    uint16_t length;        // data length（not include Length field）
    uint8_t flag;           // flag, cmd packet:0x81, response packet:0x00
    uint8_t op;             // opcode
    uint8_t sn;             // sequences number
    uint8_t state;
} cmd_response_filed_t;

typedef struct {
    uint16_t head_flag;     // header flag, fixed to 0x5A5A
    cmd_response_filed_t field;
    uint16_t crc;           // CRC check digit
} cmd_response_packet_t;

typedef struct {
    uint16_t buff_offset;
    uint16_t crc;
} push_data_t;


struct response_dfu_cfg_action_field_t {
    uint16_t length;        // data length（not include Length field）
    uint8_t flag;           // flag, cmd packet:0x81, response packet:0x00
    uint8_t op;             // opcode
    uint8_t sn;             // sequences number
    uint8_t state;
    uint8_t action;
    uint8_t response_len;
    uint8_t file_version[8];
    uint32_t file_size;
    uint16_t file_crc;
} __attribute__((packed));

struct read_dfu_cfg_response_packet_t {
    uint16_t head_flag;     // header flag, fixed to 0x5A5A
    struct response_dfu_cfg_action_field_t field;
    uint16_t crc;           // CRC check digit
} __attribute__((packed));

struct response_dist_state_field_t {
    uint16_t length;        // data length（not include Length field）
    uint8_t flag;           // flag, cmd packet:0x81, response packet:0x00
    uint8_t op;             // opcode
    uint8_t sn;             // sequences number
    uint8_t state;
    uint8_t type;
    uint8_t parameter_len;
    uint8_t dist_phase;
    uint8_t dist_state;
} __attribute__((packed));

struct dist_state_response_packet_t {
    uint16_t head_flag;     // header flag, fixed to 0x5A5A
    struct response_dist_state_field_t field;
    uint16_t crc;           // CRC check digit
} __attribute__((packed));

struct target_state_field_t {
    uint16_t addr;
    uint8_t state;
    uint8_t value;
} __attribute__((packed));

struct response_target_state_field_t {
    uint16_t length;        // data length（not include Length field）
    uint8_t flag;           // flag, cmd packet:0x81, response packet:0x00
    uint8_t op;             // opcode
    uint8_t sn;             // sequences number
    uint8_t state;
    uint8_t type;
    uint8_t response_len;
    uint8_t num;
    uint8_t target[];
} __attribute__((packed));

struct target_state_response_packet_t {
    uint16_t head_flag;     // header flag, fixed to 0x5A5A
    uint16_t length;        // data length（not include Length field）
    uint8_t flag;           // flag, cmd packet:0x81, response packet:0x00
    uint8_t op;             // opcode
    uint8_t sn;             // sequences number
    uint8_t state;
    uint8_t type;
    uint8_t response_len;
    uint8_t num;
    uint8_t target[];
} __attribute__((packed));

struct test_state_response_packet_t {
    uint16_t head_flag;     // header flag, fixed to 0x5A5A
    uint16_t length;        // data length（not include Length field）
    uint8_t flag;           // flag, cmd packet:0x81, response packet:0x00
    uint8_t op;             // opcode
    uint8_t sn;             // sequences number
    uint8_t state;
    uint8_t type;
    uint8_t response_len;
    uint8_t num;
    uint8_t target[];
} __attribute__((packed));

struct notify_dist_state_field_t {
    uint16_t length;        // data length（not include Length field）
    uint8_t flag;           // flag, cmd packet:0x81, response packet:0x00
    uint8_t op;             // opcode
    uint8_t sn;             // sequences number
    uint8_t type;
    uint8_t parameter_len;
    uint8_t dist_phase;
    uint8_t dist_state;
} __attribute__((packed));

struct dist_state_notify_packet_t {
    uint16_t head_flag;     // header flag, fixed to 0x5A5A
    struct notify_dist_state_field_t field;
    uint16_t crc;           // CRC check digit
} __attribute__((packed));

struct notify_target_state_field_t {
    uint16_t length;        // data length（not include Length field）
    uint8_t flag;           // flag, cmd packet:0x81, response packet:0x00
    uint8_t op;             // opcode
    uint8_t sn;             // sequences number
    uint8_t type;
    uint8_t parameter_len;
    uint16_t target_addr;
    uint8_t target_phase;
    uint8_t target_state;
} __attribute__((packed));

struct target_state_notify_packet_t {
    uint16_t head_flag;     // header flag, fixed to 0x5A5A
    struct notify_target_state_field_t field;
    uint16_t crc;           // CRC check digit
} __attribute__((packed));

/**************************************************************************************************
Function Declarations
**************************************************************************************************/
void parse_packet(const uint8_t *data, size_t length);
void get_mesh_dfu_adv_filter(u8 *len, u8 **data);
void set_dfu_distributor_state(uint8_t state, uint8_t val);
void notify_dfu_target_state(uint16_t addr, uint8_t phase, uint8_t state);
void notify_dfu_distributor_state(void);
void clear_sn(void);
extern struct file_parameter_t *get_file_parameter(void);

#ifdef __cplusplus
};
#endif

#endif /* MESH_DFU_APP_CMD_PROTOCOL */
