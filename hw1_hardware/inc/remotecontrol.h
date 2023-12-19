
#ifndef REMOTECONTROL_H
#define REMOTECONTROL_H

#ifdef __cplusplus
extern "C" {
#endif


#include "main.h"

#define RC_CH_VALUE_ZERO  (1024)
#define RC_CH_VALUE_RANGE (660)

#define RC_MOUSE_VALUE_ZERO  (0)
#define RC_MOUSE_VALUE_RANGE (32768)

/* ----------------------- RC Channel Definition---------------------------- */
#define RC_CH_VALUE_MIN ((uint16_t)364 )
#define RC_CH_VALUE_OFFSET ((uint16_t)1024)
#define RC_CH_VALUE_MAX ((uint16_t)1684)
/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP ((uint16_t)1)
#define RC_SW_MID ((uint16_t)3)
#define RC_SW_DOWN ((uint16_t)2)
/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_PRESSED_OFFSET_W ((uint16_t)0x01<<0)
#define KEY_PRESSED_OFFSET_S ((uint16_t)0x01<<1)
#define KEY_PRESSED_OFFSET_A ((uint16_t)0x01<<2)
#define KEY_PRESSED_OFFSET_D ((uint16_t)0x01<<3)
#define KEY_PRESSED_OFFSET_Q ((uint16_t)0x01<<4)
#define KEY_PRESSED_OFFSET_E ((uint16_t)0x01<<5)
#define KEY_PRESSED_OFFSET_SHIFT ((uint16_t)0x01<<6)
#define KEY_PRESSED_OFFSET_CTRL ((uint16_t)0x01<<7)
//#define RC_FRAME_LENGTH 18u
/* ----------------------- Data Struct ------------------------------------- */
#include "main.h"
typedef struct
{
    struct
    {
        float ch0;
        float ch1;
        float ch2;
        float ch3;
        uint8_t s1;
        uint8_t s2;
    }__attribute__((packed)) rc;
    struct
    {
        float x;
        float y;
        float z;
        uint8_t press_l;
        uint8_t press_r;
    }__attribute__((packed)) mouse;
    struct
    {
        uint16_t v;
    }__attribute__((packed)) key;
}__attribute__((packed)) RC_Ctl_t;
/* ----------------------- Internal Data ----------------------------------- */


void RemoteControlDataReceive();
void RC_init(void);
void RemoteControlDataUtilize(void);
#ifdef __cplusplus
};
#endif

#endif
