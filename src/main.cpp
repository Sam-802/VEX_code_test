#include "main.h"
#include "pros/motors.hpp"
#include "pros/imu.hpp"
#include "pros/controller.hpp"
"目前问题:给直行加距离闭环（编码器或视觉里程），以及给 opcontrol 触发加中断保护"
//硬件端口设置 
pros::MotorGroup left_drive ({-1, -2});    // 左组：1、2号口（负号=反转）
pros::MotorGroup right_drive({ 3,  4});    // 右组：3、4号口
pros::IMU        imu(21);
/*
我把端口放到21了,我感觉1-20这些端口可以主要去用于电机以及电磁阀的驱动
*/  
pros::Controller master(pros::E_CONTROLLER_MASTER);//创建一个master实例化对象,用于下文对遥控器的操作

/**
*@brief 把v限制在[lo,hi]之间
*/
//用于限制中控输出的工作电压,不让电机报废
static inline int clampi(int v, int lo, int hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

void initialize() {
    pros::lcd::initialize();//lcd液晶显示屏,这个函数的作用是初始化
    imu.reset();//这个函数用来启动校准
    uint32_t t0 = pros::millis();//计时,返回控制器从上电开机到现在经过了多少毫秒
    /*
    等 IMU 校准，最多 2 秒；IMU 没接也不会卡死
    具体逻辑:上文imu.reset()已经启动校准了,这里的imu.is_calibrating()
    用来检测是否校准这种状态,如果没有完成校准那么就返回true
    利用pros::millis()再次获取一个时间戳,其与t0相减计算时间差,如果小于2秒
    那么就停,因为imu.reset()通常只用一秒,pros::millis() - t0 < 2000设计的意图是
    如果imu坏了,防止卡死
    */ 

    while (imu.is_calibrating() && pros::millis() - t0 < 2000) {
        pros::delay(5);
    }
}

/**
*@param power_mv: ±12000，6000=半速
*@param run_ms:   跑多少毫秒
*@param P_gain:   纠偏强度 
*函数功能的具体实现:
*1.利用IMU修正偏差值
*imu.get_rotation( ) 顺时针转是正,逆时针转是负
*一开始先用imu.get_rotation()获取当前位置赋值给target(基准)
*然后在以后走直线的时候如果发生了角度的变化,那么再次执行循环体里面的imu.get_rotation()时
*会获取转过的角度,然后会与target做差,得到err,再用(err * P_gain)计算出修正值
*2.while循环的退出条件
*pros::millis()获取当前时间戳并赋值给t,进入while循环后如果时间差一旦等于
*设定的运行时间(run_ms)立即退出循环
*3.通过获得的偏差值来修正主控输出的电压,进而修正方向
*/

void drive_straight(int power_mv, int run_ms, double P_gain) {
    double target = imu.get_rotation();//读取IMU当前累计转过的角度(返回值double)
    uint32_t t    = pros::millis();

    while (pros::millis() - t < (uint32_t)run_ms) {
        double err = target - imu.get_rotation();//计算误差,再读一次当前朝向,得到期间偏离目标几度
        int fix    = (int)(err * P_gain);

        int lv = clampi(power_mv - fix, -12000, 12000);
        int rv = clampi(power_mv + fix, -12000, 12000);
        left_drive.move_voltage(lv);
        right_drive.move_voltage(rv);

        pros::delay(10);
    }
    left_drive.move_voltage(0);
    right_drive.move_voltage(0);
}
/**
*@brief 开局设定刹车模式 
*@see set_brake_mode() *@brief 传入*@param pros::E_MOTOR_BRAKE_BRAKE 让小车直接停下
*
*
*/
void autonomous() {
    left_drive.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    right_drive.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

    drive_straight(6000, 1500, 30);   // 半速走1.5秒
}

/**
*@see get_digital_new_press() *@brief 用于检测手柄上的某个按键是否按下
*/
void opcontrol() {
    while (true) {
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            drive_straight(6000, 800, 4.0);  // 按A走0.8秒
        }
        pros::delay(20);
    }
}