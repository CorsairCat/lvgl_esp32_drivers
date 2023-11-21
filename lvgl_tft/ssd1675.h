/**
 * @file ssd1675.h
 *
 */

#ifndef SSD1675_H
#define SSD1675_H

#ifdef __cplusplus
extern "C"
{
#endif

#ifdef LV_LVGL_H_INCLUDE_SIMPLE
#include "lvgl.h"
#else
#include "lvgl/lvgl.h"
#endif
#include "sdkconfig.h"

#define EPD_PANEL_WIDTH          CONFIG_LV_HOR_RES_MAX   /* 128 */
#define EPD_PANEL_HEIGHT         CONFIG_LV_VER_RES_MAX   /* 296 */

/* 128 = panel width */
#define SSD1675_COLUMNS          (EPD_PANEL_WIDTH / 8)

#define SSD1675_DC_PIN           CONFIG_LV_DISP_PIN_DC
#define SSD1675_RST_PIN          CONFIG_LV_DISP_PIN_RST
#define SSD1675_USE_RST          CONFIG_LV_DISP_USE_RST
#define SSD1675_BUSY_PIN         CONFIG_LV_DISP_PIN_BUSY
#define SSD1675_BUSY_LEVEL       1 //chip is busy if the pin level is high

/* SSD1675 commands */
#define SSD1675_CMD_GDO_CTRL			0x01 //Driver output control
#define SSD1675_CMD_GDV_CTRL			0x03
#define SSD1675_CMD_SDV_CTRL			0x04
// #define SSD1675_CMD_SOFTSTART			0x0c
// #define SSD1675_CMD_GSCAN_START			0x0f

#define SSD1675_CMD_SLEEP_MODE1			0x10 //enter deep sleep 1 - retain RAM
// #define SSD1675_CMD_SLEEP_MODE2         0x11 //enter deep sleep 2 - cannot retain RAM
//After entering Deep sleep Mode, BUSY will stay high.
//To Exit Deep Sleep a HWRESET should be sent.

#define SSD1675_CMD_ENTRY_MODE			0x11 //Data entry mode
#define SSD1675_CMD_SW_RESET			0x12 //SWRESET
#define SSD1675_CMD_READ_INT_TEMP       0x18 //Read built-in temperature sensor
#define SSD1675_CMD_TSENS_CTRL			0x1a
#define SSD1675_CMD_MASTER_ACTIVATION	0x20 //Activate Display Update Sequence
#define SSD1675_CMD_UPDATE_CTRL1		0x21 //Display update control
#define SSD1675_CMD_UPDATE_CTRL2		0x22 //Display Update Control
#define SSD1675_CMD_WRITE1_RAM			0x24 //Write Black (0) and Wite (1) image to RAM
#define SSD1675_CMD_WRITE2_RAM		    0x26 //Write RED (1) and NonRED (0) image to RAM
#define SSD1675_CMD_VCOM_SENSE			0x28
#define SSD1675_CMD_VCOM_SENSE_DURATON	0x29
#define SSD1675_CMD_PRGM_VCOM_OTP		0x2a
#define SSD1675_CMD_VCOM_VOLTAGE		0x2c
#define SSD1675_CMD_PRGM_WS_OTP			0x30
#define SSD1675_CMD_UPDATE_LUT			0x32
#define SSD1675_CMD_PRGM_OTP_SELECTION	0x36
#define SSD1675_CMD_WRITE_DISPL_OPT 	0x37
#define SSD1675_CMD_WRITE_DUMMY         0x3a
#define SSD1675_CMD_WRITE_GATELINE      0x3b
#define SSD1675_CMD_BWF_CTRL			0x3c //BorderWavefrom
#define SSD1675_CMD_RAM_XPOS_CTRL		0x44 //set Ram-X address start/end position
#define SSD1675_CMD_RAM_YPOS_CTRL		0x45 //set Ram-Y address start/end position
#define SSD1675_CMD_RAM_XPOS_CNTR		0x4e //set RAM x address count to 0;
#define SSD1675_CMD_RAM_YPOS_CNTR		0x4f //set RAM y address count to 0X199;
#define SSD1675_CMD_ANALOGBLOCK         0x74
#define SSD1675_CMD_DIGITALBLOCK        0x7E

/* Data entry sequence modes */
#define SSD1675_DATA_ENTRY_MASK			0x07
#define SSD1675_DATA_ENTRY_XDYDX		0x00
#define SSD1675_DATA_ENTRY_XIYDX		0x01
#define SSD1675_DATA_ENTRY_XDYIX		0x02
#define SSD1675_DATA_ENTRY_XIYIX		0x03
#define SSD1675_DATA_ENTRY_XDYDY		0x04
#define SSD1675_DATA_ENTRY_XIYDY		0x05
#define SSD1675_DATA_ENTRY_XDYIY		0x06
#define SSD1675_DATA_ENTRY_XIYIY		0x07

/* Options for display update */
#define SSD1675_CTRL1_INITIAL_UPDATE_LL	0x00
#define SSD1675_CTRL1_INITIAL_UPDATE_LH	0x01
#define SSD1675_CTRL1_INITIAL_UPDATE_HL	0x02
#define SSD1675_CTRL1_INITIAL_UPDATE_HH	0x03

/* Options for display update sequence */
#define SSD1675_CTRL2_ENABLE_CLK		0x80
#define SSD1675_CTRL2_ENABLE_ANALOG		0x40
#define SSD1675_CTRL2_TO_INITIAL		0x08
#define SSD1675_CTRL2_TO_PATTERN		0x04
#define SSD1675_CTRL2_DISABLE_ANALOG	0x02
#define SSD1675_CTRL2_DISABLE_CLK		0x01

#define SSD1675_SLEEP_MODE_DSM			0x01
#define SSD1675_SLEEP_MODE_PON			0x00

/* time constants in ms */
#define SSD1675_RESET_DELAY			    20 //At least 10ms delay
#define SSD1675_BUSY_DELAY			    1
// normal wait time max 20 times x 10ms
#define SSD1675_WAIT                    20

void ssd1675_init(void);
void ssd1675_flush(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map);
void ssd1675_rounder(lv_disp_drv_t * disp_drv, lv_area_t *area);
void ssd1675_set_px_cb(lv_disp_drv_t * disp_drv, uint8_t* buf, lv_coord_t buf_w, lv_coord_t x, lv_coord_t y, lv_color_t color, lv_opa_t opa);

void ssd1675_deep_sleep(void);

#ifdef __cplusplus
} /* extern "C" */
#endif


#endif /* __SSD1675_REGS_H__ */