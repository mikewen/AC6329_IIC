/*************************************************************************************************/
/*!
*  \file       mesh_distributor_loader.c
*
*  \brief      Main file for example
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

#include "model_api.h"
#include "mesh_distributor_loader.h"
#include "system/includes.h"

#define LOG_TAG "[Mesh_Dist_Loader]"
#define LOG_INFO_ENABLE
//#define LOG_DEBUG_ENABLE
#define LOG_ERROR_ENABLE
/* #define LOG_DUMP_ENABLE */
#define LOG_CLI_ENABLE
#include "mesh_log.h"

#if (CONFIG_MESH_MODEL == SIG_MESH_DFU_DISTRIBUTOR_DEMO)

/**************************************************************************************************
Macros
**************************************************************************************************/

/* 是否开启测试 */
#define             mesh_dist_loader_test_en    0
/* CRC-16-XMODEM 的多项式 */
#define             POLY                        0x1021

#define OTA_READ_FLASH_SIZE       (4096 + 16)



/**************************************************************************************************
Global Variables
**************************************************************************************************/
/* 增加一个flash配置区用于烧录target node的bin文件, 配置区在xxx_global_build_cfg.h定义 */
static const char mesh_dist_loader_path[] = "mnt/sdfile/app/mesh"; // 配置区路径(isd_config_rule.c)
static struct vfs_attr mesh_dist_loader_read_attr;
static FILE *mesh_dist_loader_fp = NULL;
static volatile mesh_dist_loader_cur_offset;
static void (*mesh_dist_loader_complete_cb)(void) = NULL;
static u8_t test_buffer[OTA_READ_FLASH_SIZE] = {0};

/**************************************************************************************************
Functions
**************************************************************************************************/
/*************************************************************************************************/
/**
 * @brief 计算 CRC-16-XMODEM 的值
 *
 * @param data      输入数据的指针
 * @param length    数据的长度
 * @param crc       初始 CRC 值
 *
 * @return          计算后的 CRC 值
 */
/*************************************************************************************************/
static uint16_t crc16_xmodem(const uint8_t *data, size_t length, uint16_t crc)
{
    for (size_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i] << 8; // 将当前字节移入CRC高位
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ POLY; // 如果最高位为1，左移并异或多项式
            } else {
                crc <<= 1; // 否则直接左移
            }
        }
    }
    return crc;
}

static bool mesh_dis_loader_info_zone_earse(u32 fsize, u32 sclust)
{
    if (!fsize || !sclust) {
        log_error("please enter the correct file size and address!");
        return false;
    }

    if (sfc_erase_zone(sclust, fsize)) {
        log_info("erase ok!");
        return true;
    } else {
        log_error("erase fail!");
        return false;
    }
}

/*************************************************************************************************/
/*!
*  \brief  获取数据指针.
*
*  \return 数据指针.
*/
/*************************************************************************************************/
int mesh_dist_loader_init(mesh_dist_loader_type wr_type)
{
    if (wr_type == W_TYPE) {
        /* 打开文件 */
        mesh_dist_loader_fp = fopen(mesh_dist_loader_path, "wr");
        if (mesh_dist_loader_fp) {
            log_debug("open succ for write");
        } else {
            log_error("open fail for write!");
            return -1;
        }
        /* 获取文件属性, fszie: 区域大小, sclust: 区域地址 */
        fget_attrs(mesh_dist_loader_fp, &mesh_dist_loader_read_attr);
        log_debug("sclust= %08x,size= %04x", mesh_dist_loader_read_attr.sclust, mesh_dist_loader_read_attr.fsize);
        /* 将cpu地址转换为flash的物理地址, 用于擦除操作 */
        mesh_dist_loader_read_attr.sclust = sdfile_cpu_addr2flash_addr(mesh_dist_loader_read_attr.sclust);
        /* 写之前先擦除所有flash内容 */
        mesh_dis_loader_info_zone_earse(mesh_dist_loader_read_attr.fsize, mesh_dist_loader_read_attr.sclust);
    } else {
        /* 打开文件 */
        mesh_dist_loader_fp = fopen(mesh_dist_loader_path, "r");
        if (mesh_dist_loader_fp) {
            log_debug("open success for read!");
        } else {
            log_error("open fail for read!");
            return -1;
        }
        /* 获取文件属性, fszie: 区域大小, sclust: 区域地址 */
        fget_attrs(mesh_dist_loader_fp, &mesh_dist_loader_read_attr);
        log_debug("sclust= %08x,size= %04x", mesh_dist_loader_read_attr.sclust, mesh_dist_loader_read_attr.fsize);
        return mesh_dist_loader_read_attr.fsize;
    }
}

/*************************************************************************************************/
/**
 * @brief 设置read loader完成回调
 *
 */
/*************************************************************************************************/
void mesh_dist_loader_set_complete_cb(void (*cb)(void))
{
    mesh_dist_loader_complete_cb = cb;
}

/*************************************************************************************************/
/**
 * @brief 关闭文件
 *
 * @return  0: 成功
 */
/*************************************************************************************************/
int mesh_dist_loader_close(void)
{
    if (mesh_dist_loader_fp) {
        fclose(mesh_dist_loader_fp);
        mesh_dist_loader_read_attr.fsize = 0;
        mesh_dist_loader_read_attr.sclust = 0;
        mesh_dist_loader_fp = NULL;
        mesh_dist_loader_cur_offset = 0;
        log_debug("close succ");
        return 0;
    }
    return -1;
}

void mesh_dist_loader_read_complete(void)
{
    log_debug(">>>func:%s", __FUNCTION__);
    mesh_dist_loader_close();
    if (mesh_dist_loader_complete_cb) {
        mesh_dist_loader_complete_cb();
    }
}

static u32 remove_trailing_ff(uint8_t *data, size_t len)
{
    /* 检查尾部是否有连续的FF FF，有说明已经到文件末尾 */
    while (len > 1 && data[len - 1] == 0xFF && data[len - 2] == 0xFF) {
        len -= 2; // 去掉尾部的FF FF
    }
    return len; // 返回去掉FF FF后的长度
}

/*************************************************************************************************/
/**
 * @brief 读取数据
 *
 * @param buffer  数据缓冲区
 * @param len     读取长度
 * @param offset  偏移量
 *
 * @return  读取数据指针
 */
/*************************************************************************************************/
int mesh_dist_read_data(void *buffer, u32 len, u32_t block_size, u32_t block_offset, u32_t chunk_offset)
{
    if (!mesh_dist_loader_fp) {
        log_error("File is not open, please init first!");
        return -1;
    }

    u32 read_len = 0;
    u32_t offset = 0;
    u16 handle_len = 0;

    if (chunk_offset == 0U) {
        offset = block_offset + chunk_offset;
        /* seek the flash addr to read. */
        if (fseek(mesh_dist_loader_fp, offset, SEEK_SET) < 0) {
            log_error("Failed to seek to offset %u", offset);
            mesh_dist_loader_close();
            return -2;
        }

        /* 检查读取长度是否超过flash划分区域大小 */
        if (offset + len > mesh_dist_loader_read_attr.fsize) {
            len = mesh_dist_loader_read_attr.fsize - offset; //调整读取长度为剩余数据量
            log_error("Adjusted read length to remaining file size: %u bytes", len);
            return -3;
        }

        /* 读取文件数据 */
        read_len = fread(mesh_dist_loader_fp, test_buffer, block_size);

        log_debug("Data read successfully. Offset: %u, read length: %u", offset, read_len);

        /* 如果未到文件末尾，移动文件指针 */
        if (!read_len) {
            log_debug(">>>>Reached the end of the file, mesh loader complete!");
            mesh_dist_loader_read_complete();
        }
    }

    memcpy(buffer, &test_buffer[chunk_offset], len);

    return 1;
}

int mesh_dist_loader_read_data(void *buffer, u32 len)
{
    if (!mesh_dist_loader_fp) {
        log_error("File is not open, please init first!");
        return -1;
    }

    u32 read_len = 0;

    /* 检查读取长度是否超过flash划分区域大小 */
    if (mesh_dist_loader_cur_offset + len > mesh_dist_loader_read_attr.fsize) {
        len = mesh_dist_loader_read_attr.fsize - mesh_dist_loader_cur_offset; // 调整读取长度为剩余数据量
        log_error("Adjusted read length to remaining file size: %u bytes", len);
        return -3;
    }

    /* 读取文件数据 */
    read_len = fread(mesh_dist_loader_fp, buffer, len);
    if (read_len != len) {
        log_error("Failed to read the full data, read_len: %u", read_len);
        mesh_dist_loader_close();
        return -1;
    }

    // 更新偏移量
    mesh_dist_loader_cur_offset += read_len;

    log_debug("Data read successfully. Offset: %u, Read length: %u", mesh_dist_loader_cur_offset, read_len);

    // 如果未到文件末尾，移动文件指针
    if (read_len) {
        if (fseek(mesh_dist_loader_fp, mesh_dist_loader_cur_offset, SEEK_SET) < 0) {
            log_error("Failed to seek to offset %u", mesh_dist_loader_cur_offset);
            mesh_dist_loader_close();
            return -2;
        }
    } else {
        log_debug(">>>>Reached the end of the file, mesh loader complete!");
        mesh_dist_loader_read_complete();
    }

    return read_len;
}

/*************************************************************************************************/
/**
 * @brief 写入数据到文件
 *
 * @param buffer  数据缓冲区
 * @param len     写入长度
 *
 * @return  实际写入的数据长度，如果出错返回-1
 */
/*************************************************************************************************/
int mesh_dist_loader_write_data(const void *buffer, u32 len)
{
    if (!mesh_dist_loader_fp) {
        log_error("File is not open, please init first!");
        return -1;
    }

    /* 移动文件指针到当前的写入偏移量 */
    if (fseek(mesh_dist_loader_fp, mesh_dist_loader_cur_offset, SEEK_SET) < 0) {
        log_error("Failed to seek to current offset %u", mesh_dist_loader_cur_offset);
        return -2;
    }

    /* 检查写入长度是否超过flash划分区域大小 */
    if (mesh_dist_loader_cur_offset + len > mesh_dist_loader_read_attr.fsize) {
        log_error("Write length exceeds allocated file size");
        return -3;
    }

    u32 write_len = fwrite(mesh_dist_loader_fp, buffer, len);
    if (write_len != len) {
        log_error("Failed to write the full data, write_len: %u", write_len);
        return -1;
    }

    /* 更新当前偏移量 */
    mesh_dist_loader_cur_offset += write_len;

    log_debug("Data written successfully. New offset: %u, Write length: %u", mesh_dist_loader_cur_offset, write_len);

    return write_len;
}

/*************************************************************************************************/
/**
 *
 *  @brief 获取当前文件指针偏移量
 *
 */
/*************************************************************************************************/
void mesh_dist_loader_get_cur_offset(u32 *offset)
{
    *offset = mesh_dist_loader_cur_offset;
}

/*************************************************************************************************/
/**
 * @brief 计算bin文件的CRC值和真实大小(去掉尾部的FF FF)
 *
 * @param crc     输出参数，返回计算的CRC值
 * @param size    输出参数，返回文件的真实大小
 *
 * @return  0: 成功, -1: 失败
 */
/*************************************************************************************************/
int mesh_dist_loader_calculate_crc(u16 *crc, u32 *size, u32 file_size)
{
    mesh_dist_loader_init(R_TYPE);
    if (!mesh_dist_loader_fp) {
        log_error("File is not open");
        return -1;
    }

    u8 buffer[512];
    u32 read_size = 0;
    u32 total_read = 0;
    u16 crc_value = 0x0000; // 初始化CRC-16-XMODE值
    size_t read_len = 0;

    while (1) {
        wdt_clear();

        if ((file_size - total_read) < 512) {
            read_size = file_size - total_read;
        } else {
            read_size = 512;
        }

        int read_len = mesh_dist_loader_read_data(buffer, read_size);

        if (read_len <= 0) {
            log_debug("read file info finish!!", read_len);
            mesh_dist_loader_close();
            break;
        }

        /* 计算CRC */
        crc_value = crc16_xmodem(buffer, read_len, crc_value);

        total_read += read_len;

        /* 读取到文件末尾 */
        if (file_size == total_read) {
            mesh_dist_loader_close();
            break;
        }
    }

    /* 设置输出值 */
    *crc = crc_value;
    *size = total_read;

    log_debug("Calculated CRC: 0x%x, True File Size: %u", crc_value, total_read);
    return 0;
}

/*************************************************************************************************/
/**
 * @brief 读取数据测试
 *
 */
/*************************************************************************************************/
#if mesh_dist_loader_test_en
static u32 mesh_dist_loader_test_timer;

void mesh_dist_loader_complete_test(void)
{
    sys_timer_del(mesh_dist_loader_test_timer);
    // u8 buf_end[3] = {0x77, 0x88, 0x98};
    // multi_client_write_data(buf_end, 3);
    log_debug("mesh_dist_loader_complete_test");
}

void mesh_dist_loader_test_read(void)
{
    u8 buffer[512];       // 数据缓冲区
    u32 chunk_size = 256; // 初始块大小设为256字节

    // 读取数据
    int read_len = mesh_dist_loader_read_data(buffer, chunk_size);

    if (read_len > 0) {
        log_info("Read chunk at offset %u:", mesh_dist_loader_cur_offset - chunk_size); // 打印当前块的起始偏移量
        log_info_hexdump(buffer, read_len);                                             // 打印读取的数据
        // multi_client_write_data(buffer, read_len);
    } else {
        sys_timer_del(mesh_dist_loader_test_timer);
        mesh_dist_loader_close();
    }
}

void mesh_dist_loader_test(void)
{
    mesh_dist_loader_init(R_TYPE); // 初始化
    mesh_dist_loader_set_complete_cb(mesh_dist_loader_complete_test); // 设置完成回调
    mesh_dist_loader_test_timer = sys_timer_add(NULL, mesh_dist_loader_test_read, 100);  // 100ms读取数据
}
#endif
#endif
