#include "GUI_Main.h"

lv_obj_t *g_MainObj;
lv_obj_t *meter;
lv_meter_indicator_t * indic;

void LVGLDemoInit(void)
{
    g_MainObj = lv_obj_create(lv_scr_act());
    lv_obj_set_pos(g_MainObj, 0, 0);
    lv_obj_set_size(g_MainObj, 800, 480);
    
    meter = lv_meter_create(g_MainObj);
    lv_obj_set_size(meter, 200, 200);
    lv_obj_set_pos(meter, 0, 0);
    
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

    /*Create an animation to set the value*/
//    lv_anim_t a;
//    lv_anim_init(&a);
//    lv_anim_set_exec_cb(&a, 0);
//    lv_anim_set_var(&a, indic);
//    lv_anim_set_values(&a, 0, 100);
//    lv_anim_set_time(&a, 2000);
//    lv_anim_set_repeat_delay(&a, 100);
//    lv_anim_set_playback_time(&a, 500);
//    lv_anim_set_playback_delay(&a, 100);
//    lv_anim_set_repeat_count(&a, LV_ANIM_REPEAT_INFINITE);
//    lv_anim_start(&a); 
}

void SetPowerValue(INT32U Value)
{
    lv_meter_set_indicator_value(meter, indic, Value);    
}

