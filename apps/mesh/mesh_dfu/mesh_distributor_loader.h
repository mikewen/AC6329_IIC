#ifndef MESH_DISTRIBUTOR_LOADER_H
#define MESH_DISTRIBUTOR_LOADER_H
#include "cpu.h"

/**************************************************************************************************
Constants
**************************************************************************************************/
typedef enum {
    R_TYPE = 0,
    W_TYPE,
} mesh_dist_loader_type;



/**************************************************************************************************
Function Declarations
**************************************************************************************************/
int mesh_dist_loader_init(mesh_dist_loader_type wr_type);
void mesh_dist_loader_set_complete_cb(void (*cb)(void));
int mesh_dist_read_data(void *buffer, u32 len, u32_t block_size, u32_t block_offset, u32_t chunk_offset);
int mesh_dist_loader_read_data(void *buffer, u32 len);
int mesh_dist_loader_write_data(const void *buffer, u32 len);
int mesh_dist_loader_close();
void mesh_dist_loader_get_cur_offset(u32 *offset);
int mesh_dist_loader_calculate_crc(u16 *crc, u32 *size, u32 file_size);

#endif
