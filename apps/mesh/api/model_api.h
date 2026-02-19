#ifndef __MODEL_API_H__
#define __MODEL_API_H__
#include "api/basic_depend.h"

//< Detail in "MshPRT_v1.1"
#define SIG_MESH_GENERIC_ONOFF_CLIENT       0  // examples/generic_onoff_client.c
#define SIG_MESH_GENERIC_ONOFF_SERVER       1  // examples/generic_onoff_server.c
#define SIG_MESH_VENDOR_CLIENT              2  // examples/vendor_client.c
#define SIG_MESH_VENDOR_SERVER              3  // examples/vendor_server.c
#define SIG_MESH_ALIGENIE_LIGHT            	4  // examples/AliGenie_light.c
#define SIG_MESH_ALIGENIE_SOCKET            5  // examples/AliGenie_socket.c
#define SIG_MESH_ALIGENIE_FAN            	6  // examples/AliGenie_fan.c
#define SIG_MESH_LIGHT_LIGHTNESS_SERVER		7  // examples/light_lightness_server.c
#define SIG_MESH_TUYA_LIGHT            	    8  // examples/TUYA_light.c
#define SIG_MESH_TENCENT_MESH               9  // examples/tecent_mesh.c
#define SIG_MESH_PROVISIONER                10 // examples/provisioner.c
#define SIG_MESH_PROVISIONEE                11 // examples/provisionee.c

#define SIG_MESH_OCCUPACY_SENSOR_NLC        12 // examples/light_occupacy_sensor_nlc.c
#define SIG_MESH_SENSOR_OBSEVER_NLC         13 // examples/light_occupacy_observer_nlc.c
#define SIG_MESH_BASIC_LIGHTNESS_CTRL_NLC   14 // examples/basic_lightness_crtl_nlc.c
#define SIG_MESH_DIMMING_CONTROL_NLC        15 // examples/dimming_control_nlc.c
#define SIG_MESH_ENERGY_MONITOR_NLC         16 // examples/energy_monitor_nlc.c
#define SIG_MESH_BASIC_SCENE_SELECTOR_NLC   17 // examples/basic_scene_selector_nlc.c
#define SIG_MESH_AMBIENT_LIGHT_SENSOR_NLC   18 // examples/ambient_light_sensor_nlc.c
#define SIG_MESH_DFU_DISTRIBUTOR_DEMO       19 // examples/dfu_distributor_demo.c
#define SIG_MESH_DFU_TARGET_DEMO            20 // examples/dfu_target_demo.c
// more...

//< Config whick example will use in <examples>
#define CONFIG_MESH_MODEL                   SIG_MESH_BASIC_LIGHTNESS_CTRL_NLC

/* Tmall Update tool */
#define TMALL_UPDATE_TOOL						0

#define BYTE_LEN(x...)                      sizeof((u8 []) {x})

#define MAC_TO_LITTLE_ENDIAN(x) \
    (x & 0xff), \
    ((x >> 8) & 0xff), \
    ((x >> 16) & 0xff), \
    ((x >> 24) & 0xff), \
    ((x >> 32) & 0xff), \
    ((x >> 40) & 0xff)

#define MAC_TO_BIG_ENDIAN(x) \
    ((x >> 40) & 0xff), \
    ((x >> 32) & 0xff), \
    ((x >> 24) & 0xff), \
    ((x >> 16) & 0xff), \
    ((x >> 8) & 0xff), \
    (x & 0xff)

void gpio_pin_write(u8_t led_index, u8_t onoff);
void led_blink_worker_on(u32 gpio);
void led_blink_worker_off(u32 gpio);
u16_t get_primary_addr(void);
void bt_mac_addr_set(u8 *bt_addr);
void prov_complete(u16_t net_idx, u16_t addr);
void prov_reset(void);
extern uint32_t btctler_get_rand_from_assign_range(uint32_t rand, uint32_t min, uint32_t max);
extern void pseudo_random_genrate(uint8_t *dest, unsigned size);
extern void ble_bqb_test_thread_init(void);
#endif /* __MODEL_API_H__ */
