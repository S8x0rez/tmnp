#include "auto_handler.h"

#include "motor.h"
#include "bluetooth.h"
#include "camera.h"
#include "stage_env.h"
#include "system.h"

#include "app.h"

#include <math.h>
#include <stdlib.h>


enum AutoHandleState {
    STAT_SEARCH,
    STAT_AUTO_MOVE,
    STAT_SWING,
    STAT_GOAL,
    STAT_WAIT
};

enum SearchEvent {
    EVENT_CAPTURE,
    EVENT_MOVE,
    EVENT_CALC
};

void set_stage(int [][STAGE_BLOCK_COL]);
void calc_route(ITEMS *, int [][STAGE_BLOCK_COL]);
void print_goal();
void adjust_color_posi(ROBOT *);

enum AutoHandleState g_auto_stat;

void auto_handler()
{
    ROBOT robot = {15, 20, -90, 0};
    OBJECT ball = {15, 28, 4, 4};
    OBJECT hole = {170, 170, 0, 0};
    
#ifdef NORMAL_STAGE
    OBJECT wall;
#else
    OBJECT wall = {0, 120, 120, 0};
#endif

    ITEMS obj = {&robot, &ball, &hole, &wall};
    
    int stage[STAGE_BLOCK_ROW][STAGE_BLOCK_COL] = {};
    int arm_angle;
    
    ev3_lcd_draw_string("Auto mode runnnig", 0, l_font_property.height * 0);

    ev3_motor_rotate(PORT_ARM_MOTOR, 180, 30, true);
    ev3_motor_reset_counts(PORT_ARM_MOTOR);
    
    bt_connect();
    
	act_tsk(CAM_TASK);
    tslp_tsk(1000);

    g_auto_stat = STAT_WAIT;
    
    while (g_sys_mode_stat == MODE_AUTO) {
        if (g_auto_stat == STAT_SWING) {
            ev3_lcd_draw_string("SWING             ", 0, l_font_property.height * 4);
            
            if (acos(hole.x * 1 + hole.y * 0) * 180 / PI - robot.direct < 0) {    // swing direction  //
                swing_task(&ball, &hole, -1);
                robot_rotate(&robot, -90);
            }
            else {
                swing_task(&ball, &hole, 1);
                robot_rotate(&robot, 90);
            }

            g_auto_stat = STAT_SEARCH;
        }
        else if (g_auto_stat == STAT_SEARCH) {
            search_task(&obj);

            // if (sqrt(pow(ball.x - hole.x, 2) + pow(ball.y - hole.y, 2))) {
            //     g_auto_stat = STAT_GOAL;
            //     continue;
            // }

            calc_route(&obj, stage);

            g_auto_stat = STAT_AUTO_MOVE;
        }
        else if (g_auto_stat == STAT_AUTO_MOVE) {
            move_task(&robot, stage);

            if (g_detected_flag == 1) {
                g_auto_stat = STAT_SWING;
            }
            else {
                g_auto_stat = STAT_SEARCH;
            }
        }
        else if(g_auto_stat == STAT_WAIT) {
            if(ev3_button_is_pressed(ENTER_BUTTON)){
                ev3_lcd_fill_rect(0, l_font_property.height * 4, EV3_LCD_WIDTH, l_font_property.height, EV3_LCD_WHITE);
                ev3_motor_rotate(PORT_ARM_MOTOR, -360, 50, true);
                arm_angle = ev3_motor_get_counts(PORT_ARM_MOTOR) % 360;
                ev3_motor_rotate(PORT_ARM_MOTOR, arm_angle, 20, true);
                robot_rotate(&robot, -90);
                g_auto_stat = STAT_SEARCH;
            }else{
                ev3_lcd_draw_string("READY?(push entry)", 0, l_font_property.height * 4);
            }
        }
        else {
            print_goal();
        }
    }

    ev3_motor_rotate(PORT_ARM_MOTOR, 180, 20, false);
}

void set_stage(int stage[][STAGE_BLOCK_COL])
{
    for (int i = 0; i < STAGE_BLOCK_ROW; i++) {
        for (int j = 0; j < STAGE_BLOCK_COL; j++) {
            stage[i][j] = (j + i * 2) % 5;
        }
    }
    // 012340123
    // 234012340
    // 401234012
    // 123401234
    // 340123401
    // 012340123
}

void calc_route(ITEMS *obj, int stage[][STAGE_BLOCK_COL])
{
    int straight = 0;
    int target_x, target_y;
    int r;
    int theta;

    if (obj->wall->width == 0 && obj->wall->height == 0) {    // wall is none //
        straight = 1;
    }
    else {  // wall is paralle to the stage //
        if (obj->wall->x == 0) {
            if (obj->ball->y < obj->wall->y) straight = 0;
            else straight = 1;
        }
        else if (obj->wall->y == 0) {
            if (obj->ball->x < obj->wall->x) straight = 0;
            else straight = 1;
        }
    }


    if (straight == 1) {
        target_x = obj->hole->x;
        target_y = obj->hole->y;
    }
    else {
        if (obj->wall->width != 0) {    // 壁が横向き
            target_x = (STAGE_WIDTH + obj->wall->width) / 2;
            target_y = obj->wall->y;
        }
        else {  // 壁が縦向き
            target_x = obj->wall->x;
            target_y = (STAGE_HEIGHT + obj->wall->height) / 2;    
        }
    }


    if (target_x == obj->ball->x) { // y = n    //
        target_x = obj->robot->x;
        target_y = obj->ball->y;

        obj->robot->route[0][1] = abs(target_x - obj->robot->x);
        obj->robot->route[1][1] = abs(obj->ball->y - target_y);
    }
    else if (target_y == obj->ball->y) {
        target_x = obj->ball->x;
        target_y = obj->robot->y;
        
        obj->robot->route[0][1] = abs(target_y - obj->robot->y);
        obj->robot->route[1][1] = abs(obj->ball->x - target_x);
    }
    else { //  y = ax + b || y = n //
        double a1 = (target_y - obj->ball->y) / (target_x - obj->ball->x); // ボールとゴールの直線  //

        double a2 = -1 / a1; // ボールを通る上記の直線の垂線    //
        double b2 = obj->ball->y - a2 * obj->ball->x;

        if (abs((a2 * obj->robot->x + b2) - obj->robot->y) > abs((obj->robot->y - b2) / a2 - obj->robot->x)) {
            target_x = obj->robot->x;
            target_y = a2 * obj->robot->x + b2;            
        }
        else {
            target_x = (obj->robot->y - b2) / a2;
            target_y = obj->robot->y;
        }
    }

    r = sqrt(pow(target_x - obj->robot->x, 2) + pow(target_y - obj->robot->y, 2));
    theta = acos((target_x * 1 + target_y * 0) / r) * 180 / PI;  // 内積から計算

    obj->robot->route[0][0] = -1 * obj->robot->direct + theta;
    obj->robot->route[0][1] = r;

    r = sqrt(pow(obj->ball->x - target_x, 2) + pow(obj->ball->y - target_y, 2));
    theta = acos(((obj->ball->x - target_x) * 1 + (obj->ball->y - target_y) * 0) / r) * 180 / PI;  // 内積から計算


		sprintf(str, "%3d , %3d , %3d", obj->ball->x, target_x, theta);
		ev3_lcd_draw_string(str, 0, l_font_property.height * 3);
    
    obj->robot->route[1][0] = -1 * obj->robot->direct - obj->robot->route[0][0] + theta; // rotate angle //
    obj->robot->route[1][1] = r;


    sprintf(str, "%3d,%3d,%3d,%3d", obj->robot->route[0][0],obj->robot->route[0][1],obj->robot->route[1][0],obj->robot->route[1][1]);
    ev3_lcd_draw_string(str, 0, 0);

    tslp_tsk(10000);
}

void print_goal()
{
    ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
    ev3_lcd_draw_string("GOAL!!", 0, l_font_property.height * 4);
}