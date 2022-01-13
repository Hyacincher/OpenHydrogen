#include "GUI_Main.h"

lv_obj_t *g_MainObj;
lv_obj_t *meter;
lv_meter_indicator_t * indic;

lv_obj_t *g_Lock;
lv_obj_t *g_Height;
lv_obj_t *g_Direction;

static void GUISetMotorLock(void);
static void GUISetHeightMode(void);
static void GUISetDirecMode(void);

void LVGLDemoInit(void)
{
    g_MainObj = lv_obj_create(lv_scr_act());
    lv_obj_set_pos(g_MainObj, 0, 0);
    lv_obj_set_size(g_MainObj, 800, 480);
    
    meter = lv_meter_create(g_MainObj);
    lv_obj_set_size(meter, 200, 200);
    lv_obj_set_pos(meter, 500, 0);
    
    /*Add a scale first*/
    lv_meter_scale_t * scale = lv_meter_add_scale(meter);
    lv_meter_set_scale_range(meter, scale, 10500, 12600, 270, 135);
    lv_meter_set_scale_ticks(meter, scale, 43, 2, 10, lv_palette_main(LV_PALETTE_GREY));
    lv_meter_set_scale_major_ticks(meter, scale, 6, 4, 15, lv_color_black(), 10);
    
    /*Add a blue arc to the start*/
    indic = lv_meter_add_arc(meter, scale, 3, lv_palette_main(LV_PALETTE_BLUE), 0);
    lv_meter_set_indicator_start_value(meter, indic, 10500);
    lv_meter_set_indicator_end_value(meter, indic, 11200);

    /*Make the tick lines blue at the start of the scale*/
    indic = lv_meter_add_scale_lines(meter, scale, lv_palette_main(LV_PALETTE_BLUE), lv_palette_main(LV_PALETTE_BLUE), false, 0);
    lv_meter_set_indicator_start_value(meter, indic, 10500);
    lv_meter_set_indicator_end_value(meter, indic, 11200);

    /*Add a red arc to the end*/
    indic = lv_meter_add_arc(meter, scale, 3, lv_palette_main(LV_PALETTE_RED), 0);
    lv_meter_set_indicator_start_value(meter, indic, 11900);
    lv_meter_set_indicator_end_value(meter, indic, 12600);

    /*Make the tick lines red at the end of the scale*/
    indic = lv_meter_add_scale_lines(meter, scale, lv_palette_main(LV_PALETTE_RED), lv_palette_main(LV_PALETTE_RED), false, 0);
    lv_meter_set_indicator_start_value(meter, indic, 11900);
    lv_meter_set_indicator_end_value(meter, indic, 12600);

    /*Add a needle line indicator*/
    indic = lv_meter_add_needle_line(meter, scale, 4, lv_palette_main(LV_PALETTE_GREY), -10);

    //lv_font_simsun_16_cjk 部分中文还没有
    
    /* 电机解锁文本 */
    g_Lock = lv_label_create(g_MainObj);
    lv_obj_set_pos(g_Lock, 0, 0);
    lv_obj_set_style_text_font(g_Lock, &lv_font_montserrat_24, 0);
    
    /* 高度模式文本 */
    g_Height = lv_label_create(g_MainObj);
    lv_obj_set_pos(g_Height, 0, 30);
    lv_obj_set_style_text_font(g_Height, &lv_font_montserrat_24, 0);
     
    /* 方向模式文本 */
    g_Direction = lv_label_create(g_MainObj);
    lv_obj_set_pos(g_Direction, 0, 60);
    lv_obj_set_style_text_font(g_Direction, &lv_font_montserrat_24, 0);
}

void GUISetPowerValue(INT32U Value)
{
    lv_meter_set_indicator_value(meter, indic, Value);    
}

void LVGLDemoTask(void)
{
    static INT32U Tick = 0;
    
    if((g_SystemTime - Tick) > 10)
    {
        GUISetMotorLock();
        GUISetHeightMode();
        GUISetDirecMode();
        Tick = g_SystemTime;
    }
}

static void GUISetMotorLock(void)
{
    if(g_DPortCtrlMsg.DPort[L_UNLOCK_MOTOR].Status.Activity)
    {
        lv_label_set_text_fmt(g_Lock, "Motor UnLock");
    }
    else
    {
        lv_label_set_text_fmt(g_Lock, "Motor Lock");
    }
}

static void GUISetHeightMode(void)
{
    switch(g_DPortCtrlMsg.HeightMode)
    {
        case UserManual:
            lv_label_set_text_fmt(g_Height, "Height Mode: UserManual");
            break;
        case FixedHeight:
            lv_label_set_text_fmt(g_Height, "Height Mode: FixedHeight");
            break;
        case AutoTakeoff:
            lv_label_set_text_fmt(g_Height, "Height Mode: AutoTakeoff");
            break;
        case AutoLanding:
            lv_label_set_text_fmt(g_Height, "Height Mode: AutoLanding");
            break;
        case AllHeightMode:
            break;
    }
}

static void GUISetDirecMode(void)
{
    switch(g_DPortCtrlMsg.DirectionMode)
    {
        case HeadDirection:
            lv_label_set_text_fmt(g_Direction, "Direc Mode: HeadDirection");
            break;
        case HeadLess:
            lv_label_set_text_fmt(g_Direction, "Direc Mode: HeadLess");
            break;
        case FixedPoint:
            lv_label_set_text_fmt(g_Direction, "Direc Mode: FixedPoint");
            break;
        case PointCruise:
            lv_label_set_text_fmt(g_Direction, "Direc Mode: PointCruise");
            break;
        case AllDirectMode:
            break;
    }    
}