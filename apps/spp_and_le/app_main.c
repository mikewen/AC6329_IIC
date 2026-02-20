/*********************************************************************************************
    *   Filename        : app_main.c

    *   Description     :

    *   Copyright:(c)JIELI  2011-2019  @ , All Rights Reserved.
*********************************************************************************************/
#include "system/includes.h"
#include "app_config.h"
#include "app_action.h"
#include "app_main.h"
#include "update.h"
#include "update_loader_download.h"
#include "app_charge.h"
#include "app_power_manage.h"
#include "asm/charge.h"


#if TCFG_KWS_VOICE_RECOGNITION_ENABLE
#include "jl_kws/jl_kws_api.h"
#endif /* #if TCFG_KWS_VOICE_RECOGNITION_ENABLE */

#define LOG_TAG_CONST       APP
#define LOG_TAG             "[APP]"
#define LOG_ERROR_ENABLE
#define LOG_DEBUG_ENABLE
#define LOG_INFO_ENABLE
/* #define LOG_DUMP_ENABLE */
#define LOG_CLI_ENABLE
#include "debug.h"

#include "asm/iic_hw.h"
#define MMC5603_ADDR        0x30
#define MMC5603_REG_XOUT0   0x00
#define MMC5603_REG_ODR    0x1A
#define MMC5603_REG_CTRL0  0x1B
#define MMC5603_REG_CTRL1  0x1C
#define MMC5603_REG_CTRL2  0x1D // Required to enable CMM
#define MMC5603_CTRL0_TM    0x01   // Trigger one-shot measurement
//#define MMC5603_CTRL0_TM    0x03   // Read temp as well, not working for continues mode
//#define RAW_DATA_LEN         9      // X, Y, Z as 24-bit each
#define RAW_DATA_LEN         6      // Only read 16 bits to fit inside BLE 20 bytes payload
//Update MMC5603_RAW_SIZE in ble_trans.c as well

// Initialize I2C as master
hw_iic_dev i2c_dev = 0; // Use I2C hardware instance 0


/*任务列表   */
const struct task_info task_info_table[] = {
#if CONFIG_APP_FINDMY
    {"app_core",            1,     0,   640 * 2,   128  },
#else
    {"app_core",            1,     0,   640,   128  },
#endif

    {"sys_event",           7,     0,   256,   0    },
    {"btctrler",            4,     0,   512,   256  },
    {"btencry",             1,     0,   512,   128  },
    {"btstack",             3,     0,   768,   256   },
    {"systimer",		    7,	   0,   128,   0	},
    {"update",				1,	   0,   512,   0    },
    {"dw_update",		 	2,	   0,   256,   128  },
#if (RCSP_BTMATE_EN)
    {"rcsp_task",		    2,	   0,   640,	0},
#endif
#if(USER_UART_UPDATE_ENABLE)
    {"uart_update",	        1,	   0,   256,   128	},
#endif
#if (XM_MMA_EN)
    {"xm_mma",   		    2,	   0,   640,   256	},
#endif
    {"usb_msd",           	1,     0,   512,   128  },
#if TCFG_AUDIO_ENABLE
    {"audio_dec",           3,     0,   768,   128  },
    {"audio_enc",           4,     0,   512,   128  },
#endif/*TCFG_AUDIO_ENABLE*/
#if TCFG_KWS_VOICE_RECOGNITION_ENABLE
    {"kws",                 2,     0,   256,   64   },
#endif /* #if TCFG_KWS_VOICE_RECOGNITION_ENABLE */
#if (TUYA_DEMO_EN)
    {"user_deal",           2,     0,   512,   512  },//定义线程 tuya任务调度
#endif
#if (CONFIG_APP_HILINK)
    {"hilink_task",         2,     0,   1024,   0},//定义线程 hilink任务调度
#endif
    {0, 0},
};

APP_VAR app_var;

void app_var_init(void)
{
    app_var.play_poweron_tone = 1;

    app_var.auto_off_time =  TCFG_AUTO_SHUT_DOWN_TIME;
    app_var.warning_tone_v = 340;
    app_var.poweroff_tone_v = 330;
}

__attribute__((weak))
u8 get_charge_online_flag(void)
{
    return 0;
}

void clr_wdt(void);
void check_power_on_key(void)
{
#if TCFG_POWER_ON_NEED_KEY

    u32 delay_10ms_cnt = 0;
    while (1) {
        clr_wdt();
        os_time_dly(1);

        extern u8 get_power_on_status(void);
        if (get_power_on_status()) {
            log_info("+");
            delay_10ms_cnt++;
            if (delay_10ms_cnt > 70) {
                /* extern void set_key_poweron_flag(u8 flag); */
                /* set_key_poweron_flag(1); */
                return;
            }
        } else {
            log_info("-");
            delay_10ms_cnt = 0;
            log_info("enter softpoweroff\n");
            power_set_soft_poweroff();
        }
    }
#endif
}

int retval = -1;

uint16_t readMS = 200;
// Definition (storage allocated here)
volatile uint8_t mmc5603_raw[RAW_DATA_LEN];
volatile uint8_t sensor_valid = 0;

// Write one byte to a register
static int mmc5603_write_reg(u8 reg, u8 data) {
    hw_iic_start(i2c_dev);
    if (!hw_iic_tx_byte(i2c_dev, (MMC5603_ADDR << 1) | 0)) { // Write bit
        hw_iic_stop(i2c_dev);
        return -1;
    }
    if (!hw_iic_tx_byte(i2c_dev, reg)) {
        hw_iic_stop(i2c_dev);
        return -1;
    }
    if (!hw_iic_tx_byte(i2c_dev, data)) {
        hw_iic_stop(i2c_dev);
        return -1;
    }
    hw_iic_stop(i2c_dev);
    return 0;
}

// Read multiple bytes starting from a register
static int mmc5603_read_regs(u8 reg, u8 *buf, int len) {
    hw_iic_start(i2c_dev);
    // Write register address
    if (!hw_iic_tx_byte(i2c_dev, (MMC5603_ADDR << 1) | 0)) {
        hw_iic_stop(i2c_dev);
        return -1;
    }
    if (!hw_iic_tx_byte(i2c_dev, reg)) {
        hw_iic_stop(i2c_dev);
        return -1;
    }
    // Repeated start (use stop/start if restart not supported)
    hw_iic_stop(i2c_dev);
    hw_iic_start(i2c_dev);
    // Send read address
    if (!hw_iic_tx_byte(i2c_dev, (MMC5603_ADDR << 1) | 1)) {
        hw_iic_stop(i2c_dev);
        return -1;
    }
    // Read data
    int read_len = hw_iic_read_buf(i2c_dev, buf, len);
    hw_iic_stop(i2c_dev);
    return read_len;
}

static int mmc5603_read_raw(u8 *raw) {
    if (mmc5603_write_reg(MMC5603_REG_CTRL0, MMC5603_CTRL0_TM) != 0)
        return -1;
    delay_2ms(2);  // Wait for conversion
    //os_time_dly(1);
    return mmc5603_read_regs(MMC5603_REG_XOUT0, raw, RAW_DATA_LEN);
}

void process_mmc5603_full(uint8_t *raw) {
    clr_wdt();

    uint16_t ux = ((uint16_t)raw[0] << 8) | raw[1];
    uint32_t uy = ((uint16_t)raw[2] << 8) | raw[3];
    uint32_t uz = ((uint16_t)raw[4] << 8) | raw[5];

    // Math: If you shifted 20-bit down to 16-bit (dropped 4 bits),
    // the sensitivity becomes 1.0 mG per LSB.
    int x_mg = (ux - 32768);
    int y_mg = (uy - 32768);
    int z_mg = (uz - 32768);

    clr_wdt();

    printf("X:%d Y:%d Z:%d mG \r\n", x_mg, y_mg, z_mg);

    /*  20 bit read
    // 1. Reconstruct 20-bit integers (Verified Logic)
    // X: Bytes 0, 1 + Top 4 bits of Byte 6
    uint32_t ux = ((uint32_t)raw[0] << 12) | ((uint32_t)raw[1] << 4) | (raw[6] >> 4);
    // Y: Bytes 2, 3 + Top 4 bits of Byte 7
    uint32_t uy = ((uint32_t)raw[2] << 12) | ((uint32_t)raw[3] << 4) | (raw[7] >> 4);
    // Z: Bytes 4, 5 + Top 4 bits of Byte 8
    uint32_t uz = ((uint32_t)raw[4] << 12) | ((uint32_t)raw[5] << 4) | (raw[8] >> 4);
    */

    /*
    // 2. Convert to signed mG (Offset 524288, Resolution 0.0625)
    //float x_mg = ((float)ux - 524288.0f) * 0.0625f;
    //float y_mg = ((float)uy - 524288.0f) * 0.0625f;
    //float z_mg = ((float)uz - 524288.0f) * 0.0625f;
    clr_wdt();

    // 3. Temperature (Register 0x09 is the 10th byte in the burst)
    // Formula per [MEMSIC Datasheet](https://memsic.com): Temp = (Raw * 0.8) - 75
    //uint8_t raw_temp = raw[9];
    //float temp_c = ((float)raw_temp * 0.8f) - 75.0f;

    // 4. One-line Print (Handling float formatting manually for safety)
    printf("X:%ld.%02u Y:%ld.%02u Z:%ld.%02u mG \r\n",
           (long)x_mg, (unsigned int)(uint32_t)(x_mg > 0 ? x_mg * 100 : -x_mg * 100) % 100,
           (long)y_mg, (unsigned int)(uint32_t)(y_mg > 0 ? y_mg * 100 : -y_mg * 100) % 100,
           (long)z_mg, (unsigned int)(uint32_t)(z_mg > 0 ? z_mg * 100 : -z_mg * 100) % 100);
    */

}

void process_mmc5603_with_temp(uint8_t *raw) {
    clr_wdt();

    // 1. Reconstruct 20-bit integers (Verified Logic)
    // X: Bytes 0, 1 + Top 4 bits of Byte 6
    uint32_t ux = ((uint32_t)raw[0] << 12) | ((uint32_t)raw[1] << 4) | (raw[6] >> 4);
    // Y: Bytes 2, 3 + Top 4 bits of Byte 7
    uint32_t uy = ((uint32_t)raw[2] << 12) | ((uint32_t)raw[3] << 4) | (raw[7] >> 4);
    // Z: Bytes 4, 5 + Top 4 bits of Byte 8
    uint32_t uz = ((uint32_t)raw[4] << 12) | ((uint32_t)raw[5] << 4) | (raw[8] >> 4);

    // 2. Convert to signed mG (Offset 524288, Resolution 0.0625)
    float x_mg = ((float)ux - 524288.0f) * 0.0625f;
    float y_mg = ((float)uy - 524288.0f) * 0.0625f;
    float z_mg = ((float)uz - 524288.0f) * 0.0625f;
    clr_wdt();


    uint8_t status = 0;
    uint8_t raw_t = 0;
    // 2. Ensure CMM is OFF (Register 0x1D = 0x00)
    mmc5603_write_reg(0x1D, 0x00);
    delay_2ms(1);

    // 3. Trigger Temperature ONLY (Register 0x1B = 0x02)
    mmc5603_write_reg(0x1B, 0x02);

    // 4. Wait a long time for the first conversion (40ms)
    delay_2ms(5);
    clr_wdt();

    // 5. Read Status AND Temp
    mmc5603_read_regs(0x18, &status, 1);
    mmc5603_read_regs(0x09, &raw_t, 1);

    // 3. Temperature (Register 0x09 is the 10th byte in the burst)
    // Formula per [MEMSIC Datasheet](https://memsic.com): Temp = (Raw * 0.8) - 75
    //int temp_c = (int)raw_t - 95;
    int temp_c = (int)raw_t*0.8 - 75;

    // 4. One-line Print (Handling float formatting manually for safety)
    printf("X:%ld.%02u Y:%ld.%02u Z:%ld.%02u mG | T:%d C\r\n",
           (long)x_mg, (unsigned int)(uint32_t)(x_mg > 0 ? x_mg * 100 : -x_mg * 100) % 100,
           (long)y_mg, (unsigned int)(uint32_t)(y_mg > 0 ? y_mg * 100 : -y_mg * 100) % 100,
           (long)z_mg, (unsigned int)(uint32_t)(z_mg > 0 ? z_mg * 100 : -z_mg * 100) % 100,
           temp_c);
}


float test_mmc5603_temp_hard_reset(void) {
    uint8_t status = 0;
    uint8_t raw_t = 0;

    // 1. Software Reset the entire chip (Register 0x1C = 0x80)
    // This clears any stuck CMM or ADC states
    //mmc5603_write_reg(0x1C, 0x80);
    //delay_2ms(10); // Wait 20ms for reboot
    //clr_wdt();

    // 2. Ensure CMM is OFF (Register 0x1D = 0x00)
    mmc5603_write_reg(0x1D, 0x00);
    delay_2ms(1);

    // 3. Trigger Temperature ONLY (Register 0x1B = 0x02)
    mmc5603_write_reg(0x1B, 0x02);

    // 4. Wait a long time for the first conversion (40ms)
    delay_2ms(5);
    clr_wdt();

    // 5. Read Status AND Temp
    mmc5603_read_regs(0x18, &status, 1);
    mmc5603_read_regs(0x09, &raw_t, 1);

    int temp_c = (int)raw_t - 95;
    //printf("DEBUG - Status: 0x%02X, Raw Temp: 0x%02X\n", status, raw_t);
    printf("T:%d C\r\n", temp_c);

    if (raw_t == 0) return -1.0f;
    return ((float)raw_t * 0.8f) - 75.0f;
}

/*
// ble_multi_profile.h
extern u16 trans_con_handle;
#define ATT_CHARACTERISTIC_ae02_01_CLIENT_CONFIGURATION_HANDLE 0x0009
#define ATT_CHARACTERISTIC_ae02_01_VALUE_HANDLE 0x0008
//#define ATT_OP_AUTO_READ_CCC = 0
//#define ATT_OP_NOTIFY = 1
static void trans_send_sensor_data(void)
{
    // Must have a valid connection
    if (!trans_con_handle) {
        return;
    }
    // Check if notifications are enabled for characteristic ae02_01
    if (ble_gatt_server_characteristic_ccc_get(trans_con_handle,
            ATT_CHARACTERISTIC_ae02_01_CLIENT_CONFIGURATION_HANDLE) != 1) {
        return;
    }

    // ble_comm_att_send_data copies the data, so it's safe to pass mmc5603_raw directly
    int ret = ble_comm_att_send_data(trans_con_handle,
                                     ATT_CHARACTERISTIC_ae02_01_VALUE_HANDLE,
                                     (uint8_t *)mmc5603_raw,
                                     RAW_DATA_LEN,
                                     0);
    if (ret) {
        log_info("sensor notify failed\n");
    }else{
        printf("sent");
    }
    clr_wdt(); os_time_dly(1);
}
*/

static void sensor_timer_cb(void *priv)
{
    u8 raw[RAW_DATA_LEN];
    int ret = mmc5603_read_raw(raw);   // Your existing read function
    //int ret = mmc5603_read_raw_CMM(raw);

    extern void wdt_clear();
    wdt_clear();    // same as? clr_wdt();
    if (ret == RAW_DATA_LEN) {
        memcpy(mmc5603_raw, raw, RAW_DATA_LEN);
        sensor_valid = 1;
        //printf("Sensor: %02x %02x %02x...\n", raw[0], raw[1], raw[2]);
        //printf("RAW: %02X %02X %02X %02X %02X %02X | %02X %02X %02X | TEMP:%02X\r\n", raw[0], raw[1], raw[2], raw[3], raw[4], raw[5], raw[6], raw[7],raw[8], raw[9]);
        //printf("RAW: %02X %02X %02X %02X %02X %02X | %02X %02X %02X \r\n", raw[0], raw[1], raw[2], raw[3], raw[4], raw[5], raw[6], raw[7],raw[8]);
        process_mmc5603_full(raw);
        //test_mmc5603_temp_hard_reset();
        //process_mmc5603_with_temp(raw);
    } else {
        sensor_valid = 0;             // Mark data as stale on error
        printf("Sensor read failed, ret=%d\n", ret);
    }
    // No need to signal anything; data is just updated
}


void app_main()
{

    retval = hw_iic_init(i2c_dev);
    if (retval != 0) {
        printf("hw_iic_init failed with code %d\n", retval);
        // Optionally, blink an LED or halt
    } else {
        printf("hw_iic_init OK\n");
        /*
        // Set Bandwidth to BW=00 (8ms measurement time / ~10Hz LPF)
        // Register 0x1C bits [1:0] = 00
        mmc5603_write_reg(MMC5603_REG_CTRL1, 0x00);
        clr_wdt();os_time_dly(1);

        // Enable Auto Set/Reset: Register 0x1B bit [5] = 1 (Auto_SR_en)
        // Note: 0x20 is the hex value for bit 5.
        mmc5603_write_reg(MMC5603_REG_CTRL0, 0x20);
        clr_wdt();os_time_dly(1);
        // Set the Data Rate (ODR) Value = frequency in Hz. e.g., 0x0A = 10Hz, 0x64 = 100Hz.
        mmc5603_write_reg(MMC5603_REG_ODR, 0x0A);
        clr_wdt();os_time_dly(1);

        // Enable Continuous Measurement Mode (CMM_en)
        // Set bit 4 of CTRL0 (0x10) AND bit 5 (0x20) for Auto_SR_en as discussed.
        mmc5603_write_reg(MMC5603_REG_CTRL0, 0x30); // 0x10 | 0x20
        //mmc5603_write_reg(MMC5603_REG_CTRL0, 0xB0);
        clr_wdt();os_time_dly(1);
        // Trigger the start of Continuous Mode
        // Set bit 4 of CTRL2 (0x10) to '1' to start the internal clock
        mmc5603_write_reg(MMC5603_REG_CTRL2, 0x10);
        clr_wdt();os_time_dly(1);
        */
        // 1. Set Bandwidth to BW=01 (~4ms measurement time)
        mmc5603_write_reg(MMC5603_REG_CTRL1, 0x01);
        os_time_dly(1);

        // 2. Disable CMM in CTRL2 (Set bit 4 to 0)
        mmc5603_write_reg(MMC5603_REG_CTRL2, 0x00);
        os_time_dly(1);

        // 3. Configure CTRL0: Enable Auto Set/Reset (0x20) but keep CMM bits (0x10, 0x80) OFF
        mmc5603_write_reg(MMC5603_REG_CTRL0, 0x20);
        os_time_dly(1);
    }

    void *timer_handle = NULL;
    timer_handle = sys_timer_add(NULL, sensor_timer_cb, readMS); // 200 ms period
    //timer_handle = sys_hi_timer_add(NULL, sensor_timer_cb, readMS); // 200 ms period
    //timer_handle = sys_timer_add(NULL, sensor_timer_cb, 200); // 20 Hz, 50 ms period
    if (timer_handle == NULL) {
        retval = 11;
        printf("Failed to create sensor timer");
    }

    struct intent it;

    if (!UPDATE_SUPPORT_DEV_IS_NULL()) {
        int update = 0;
        update = update_result_deal();
    }

    //printf(">>>>>>>>>>>>>>>>>app_main...\n");
    //printf(">>> v220,2022-11-23 >>>\n");

    if (get_charge_online_flag()) {
#if(TCFG_SYS_LVD_EN == 1)
        vbat_check_init();
#endif
    } else {
        check_power_on_voltage();
    }

#if TCFG_POWER_ON_NEED_KEY
    check_power_on_key();
#endif

#if TCFG_AUDIO_ENABLE
    extern int audio_dec_init();
    extern int audio_enc_init();
    audio_dec_init();
    audio_enc_init();
#endif/*TCFG_AUDIO_ENABLE*/

#if TCFG_KWS_VOICE_RECOGNITION_ENABLE
    jl_kws_main_user_demo();
#endif /* #if TCFG_KWS_VOICE_RECOGNITION_ENABLE */

    init_intent(&it);

#if CONFIG_APP_SPP_LE
    it.name = "spp_le";
    it.action = ACTION_SPPLE_MAIN;

#elif CONFIG_APP_AT_COM || CONFIG_APP_AT_CHAR_COM
    it.name = "at_com";
    it.action = ACTION_AT_COM;

#elif CONFIG_APP_DONGLE
    it.name = "dongle";
    it.action = ACTION_DONGLE_MAIN;

#elif CONFIG_APP_MULTI
    it.name = "multi_conn";
    it.action = ACTION_MULTI_MAIN;

#elif CONFIG_APP_NONCONN_24G
    it.name = "nonconn_24g";
    it.action = ACTION_NOCONN_24G_MAIN;

#elif CONFIG_APP_HILINK
    it.name = "hilink";
    it.action = ACTION_HILINK_MAIN;

#elif CONFIG_APP_LL_SYNC
    it.name = "ll_sync";
    it.action = ACTION_LL_SYNC;

#elif CONFIG_APP_TUYA
    it.name = "tuya";
    it.action = ACTION_TUYA;

#elif CONFIG_APP_CENTRAL
    it.name = "central";
    it.action = ACTION_CENTRAL_MAIN;

#elif CONFIG_APP_DONGLE
    it.name = "dongle";
    it.action = ACTION_DONGLE_MAIN;

#elif CONFIG_APP_BEACON
    it.name = "beacon";
    it.action = ACTION_BEACON_MAIN;

#elif CONFIG_APP_IDLE
    it.name = "idle";
    it.action = ACTION_IDLE_MAIN;

#elif CONFIG_APP_CONN_24G
    it.name = "conn_24g";
    it.action = ACTION_CONN_24G_MAIN;

#elif CONFIG_APP_FINDMY
    it.name = "findmy";
    it.action = ACTION_FINDMY;

#elif CONFIG_APP_FTMS
    it.name = "ftms";
    it.action = ACTION_FTMS;

#else
    while (1) {
        printf("no app!!!");
    }
#endif


    log_info("run app>>> %s", it.name);
    log_info("%s,%s", __DATE__, __TIME__);

    start_app(&it);

#if TCFG_CHARGE_ENABLE
    set_charge_event_flag(1);
#endif
}

/*
 * app模式切换
 */
void app_switch(const char *name, int action)
{
    struct intent it;
    struct application *app;

    log_info("app_exit\n");

    init_intent(&it);
    app = get_current_app();
    if (app) {
        /*
         * 退出当前app, 会执行state_machine()函数中APP_STA_STOP 和 APP_STA_DESTORY
         */
        it.name = app->name;
        it.action = ACTION_BACK;
        start_app(&it);
    }

    /*
     * 切换到app (name)并执行action分支
     */
    it.name = name;
    it.action = action;
    start_app(&it);
}

int eSystemConfirmStopStatus(void)
{
    /* 系统进入在未来时间里，无任务超时唤醒，可根据用户选择系统停止，或者系统定时唤醒(100ms) */
    //1:Endless Sleep
    //0:100 ms wakeup
    /* log_info("100ms wakeup"); */
    return 1;
}

__attribute__((used)) int *__errno()
{
    static int err;
    return &err;
}


