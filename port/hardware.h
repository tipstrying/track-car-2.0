#ifndef __HARDWARE_H
#define __HARDWARE_H
#ifdef __cplusplus
extern "C" {
#endif

/*输入端口映射*/

#define IN_Emergency_Port   IN_1_GPIO_Port
#define IN_Emergency_Pin    IN_1_Pin

#define IN_OnX_Port  IN_3_GPIO_Port    //上限位板载
#define IN_OnX_Pin   IN_3_Pin

#define IN_OnY_Port  IN_4_GPIO_Port    //下限位板载
#define IN_OnY_Pin   IN_4_Pin

#define IN_OnXY_Port    IN_2_GPIO_Port   //中间位板载
#define IN_OnXY_Pin     IN_2_Pin

//定位检测端口
#define IN_Y2_Port   IN_6_GPIO_Port
#define IN_Y2_Pin    IN_6_Pin

#define IN_Y1_Port   IN_5_GPIO_Port
#define IN_Y1_Pin    IN_5_Pin

#define IN_X1_Port   IN_8_GPIO_Port
#define IN_X1_Pin    IN_8_Pin

#define IN_X2_Port   IN_7_GPIO_Port
#define IN_X2_Pin    IN_7_Pin

//侧边检测端口
#define IN_Hool_Detect_L1_Port IN_9_GPIO_Port
#define IN_Hool_Detect__L1_Pin IN_9_Pin

#define IN_Hool_Detect_L2_Port IN_10_GPIO_Port
#define IN_Hool_Detect__L2_Pin IN_10_Pin

#define IN_Hool_Detect_R1_Port IN_11_GPIO_Port
#define IN_Hool_Detect__R1_Pin IN_11_Pin

#define IN_Hool_Detect_R2_Port IN_12_GPIO_Port
#define IN_Hool_Detect__R2_Pin IN_12_Pin

//防撞检测端口
#define IN_pre_L1_Port IN_13_GPIO_Port
#define IN_pre_L1_Pin IN_13_GPIO_Pin

#define IN_pre_L2_Port IN_14_GPIO_Port
#define IN_pre_L2_Pin IN_14_GPIO_Pin

#define IN_pre_L3_Port IN_15_GPIO_Port
#define IN_pre_L3_Pin IN_15_GPIO_Pin

#define IN_pre_L4_Port IN_16_GPIO_Port
#define IN_pre_L4_Pin IN_16_GPIO_Pin

#define IN_Power_Port   IN_18_GPIO_Port     //手动关机
#define IN_Power_Pin   IN_18_Pin

#define IN_Charge_Port IN_19_GPIO_Port      //检测与充电站接触是否良好
#define IN_Charge_Pin  IN_19_Pin

//输出端口映射
#define OUT_Charge_C_Port   OUT_1_GPIO_Port
#define OUT_Charge_C_Pin    OUT_1_Pin

#define OUT_LED_KEY_Port    OUT_2_GPIO_Port
#define OUT_LED_KEY_Pin     OUT_2_Pin


typedef enum {
    InOutSwitchUnknow = 0,
    InOutSwitchIn = 1,
    InOutSwitchOut = 2,
} InOutSwitch;

typedef enum {
    BeltFront = 0,
    BeltRev = 1
} BeltDirectionDef;

typedef enum {
    on = 1,
    off = 0
} OnOffDef;

OnOffDef getEmergencyKey();
OnOffDef getPowerKey();
int beltCtrl( int isRun, BeltDirectionDef dir, int speed );
void setPowerKey( OnOffDef status );
OnOffDef getThingSensor( int ID );
void setExti( int enable );
OnOffDef setChargeKey( OnOffDef status );
OnOffDef setChargeSensor( OnOffDef status );
OnOffDef getChargeKey();
InOutSwitch getSwitchStatus();
InOutSwitch setSwitch( InOutSwitch target );

#ifdef __cplusplus
}
#endif
#endif