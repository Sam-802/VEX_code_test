#include "main.h"
#include "pros/motors.hpp"
#include "pros/imu.hpp"
#include "pros/misc.hpp"
// 目前问题:给直行加距离闭环（编码器或视觉里程）目前是时间闭环，以及给 opcontrol 触发加中断保护
//硬件端口设置 
pros::MotorGroup right_drive({ 1,  2, -3});   // 右组：1、2、3号口 
pros::MotorGroup left_drive ({-8,  9,  10});  // 左组：8、9、10号口
pros::MotorGroup lift({11, 20});              // 抬升：双边驱动，11、20号口
/*
六电机底盘,符号是"每端口"级别的:
装配图里标白的是 3、9、10 三个电机,它们与同侧同伴反着转、与对侧电机同向,
所以右组里 3 取负、左组里 9 和 10 取正(左组基准 8 取负,与右组镜像)。
复核方法:把车抬起来,按住遥控器"上"键(验轮测试,六电机同符号正转):
- 六个轮子都朝前进方向转 => 符号全对;
- 某个轮子反着转 => 翻转它端口号的符号;
- 六个全反 => 整组符号全部翻一遍(前进方向定义反了)。
*/
/*
我把IMU端口放到21了,我感觉1-20这些端口可以主要去用于电机以及电磁阀的驱动
*/  
pros::IMU        imu(21);
pros::Controller master(pros::E_CONTROLLER_MASTER);//创建一个master实例化对象,用于下文对遥控器的操作

//---- 可调参数 ----
static constexpr int    DRIVE_MV        = 6000;   // 直行电压（半速）
static constexpr int    STRAIGHT_MS     = 1500;   // 每段直行时长
static constexpr double STRAIGHT_P      = 30;     // 直行纠偏强度
static constexpr double TURN_DEG        = 90.0;   // 左转角度
static constexpr double TURN_P          = 100;    // 转弯比例强度--度数与电压的汇率
static constexpr int    TURN_MIN_MV     = 2500;   // 转弯最低电压（克服静摩擦）
static constexpr double TURN_TOL_DEG    = 1.5;    // 转弯到位容差
static constexpr int    TURN_TIMEOUT_MS = 2500;   // 转弯超时保护
static constexpr int    TURN_FB_MV      = 6000;   // 无IMU时开环pivot电压
static constexpr int    TURN_FB_MS      = 1100;   // 无IMU时开环pivot时长,实车标定90度(不够就加大)
static constexpr int    LIFT_MV         = 12000;  // 抬升电压
static constexpr int    LIFT_UP_SIGN    = 1;      // 若遥控L1抬起时实际在下落,改成-1
static constexpr int    LIFT_UP_MS      = 1200;   // 抬起时长
static constexpr int    LIFT_DOWN_MS    = 1200;   // 下落时长
static constexpr int    LIFT_DOWN_WAIT_CAP_MS = 5000; // 等下落通知的上限,防死等
// 主流程进入第二段直行时写入的"开始下落"绝对时间戳; 0=还没通知
static volatile uint32_t g_lift_down_at = 0;
// IMU是否真的在线; 没装IMU时get_rotation()返回INFINITY, INF-INF=NaN会把电机电压算成垃圾值
static bool imu_ok = false;

/**
*@brief 把v限制在[lo,hi]之间
*/
static inline int clampi(int v, int lo, int hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

void initialize() {
    pros::lcd::initialize();//lcd液晶显示屏,这个函数的作用是初始化
    imu.reset();//这个函数用来启动校准
    lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);//抬升断电后靠保持刹车停住,防止结构被重力拽下来
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
    imu_ok = (imu.get_rotation() != PROS_ERR_F);//没装IMU时返回INFINITY,据此降级成开环
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
*注意修正方向:err<0 表示车顺时针(向右)偏了,要回正必须左轮慢、右轮快,
*所以 fix 要"左加右减"。原代码写成"左减右加",是正反馈,会越偏越厉害
*/

void drive_straight(int power_mv, int run_ms, double P_gain) {
    double target = imu.get_rotation();//读取IMU当前累计转过的角度
    uint32_t t    = pros::millis();

    while (pros::millis() - t < (uint32_t)run_ms) {
        double err = target - imu.get_rotation();//计算误差,再读一次当前朝向,得到期间偏离目标几度
        int fix    = imu_ok ? (int)(err * P_gain) : 0;//没IMU时err是NaN,必须关掉纠偏

        int lv = clampi(power_mv + fix, -12000, 12000);
        int rv = clampi(power_mv - fix, -12000, 12000);
        left_drive.move_voltage(lv);
        right_drive.move_voltage(rv);

        pros::delay(10);
    }
    left_drive.move_voltage(0);
    right_drive.move_voltage(0);
}

/**
*@brief 原地pivot左转,IMU角度闭环
*@param deg:        左转角度,左转=逆时针=IMU读数减小,所以目标=当前-deg
*@param P_gain:     比例强度
*@param min_mv:     最低电压,克服静摩擦让最后几度也能转完
*@param tol_deg:    到位容差,误差小于它就算转完
*@param timeout_ms: 超时保护,防止IMU异常时死循环
*/
void turn_left(double deg, double P_gain, int min_mv, double tol_deg, int timeout_ms) {
    if (!imu_ok) {//没IMU: 开环定时pivot, 靠TURN_FB_MS标定90度
        left_drive.move_voltage(-TURN_FB_MV);
        right_drive.move_voltage(TURN_FB_MV);
        pros::delay(TURN_FB_MS);
        left_drive.move_voltage(0);
        right_drive.move_voltage(0);
        return;
    }
    double target = imu.get_rotation() - deg;
    /*
    为什么要写成imu.get_rotation() - deg而不是 -deg呢
    因为我们这里做的是相对转弯,是"此时的朝向"再转过一定的角度
    如果直行漂了3°,那么imu.get_rotation()就会返回3°,我们要左转90°,那么目标就是3°-90°=-87°
    那就转87°就好了
    */
    uint32_t t    = pros::millis();//记录好当前时间戳

    while (pros::millis() - t < (uint32_t)timeout_ms) {
        double err = target - imu.get_rotation();//err从-deg逐渐趋近0
        if (err > -tol_deg) break;               //转够了(含轻微过冲)就停
        //计算修正电压
        int mv = clampi((int)(-err * P_gain), min_mv, 12000);//err<0,所以-err>0,所以mv>0,左轮反转、右轮正转
        left_drive.move_voltage(-mv);            //左轮反转
        right_drive.move_voltage(mv);            //右轮正转 => 逆时针=左转
        pros::delay(10);
    }
    left_drive.move_voltage(0);
    right_drive.move_voltage(0);
}

/**
*@brief 抬升"抬起→保持→下落"一次性时序,放在独立任务里与底盘并发跑,跑完自行退出
*时间线:autonomous开始即抬起(第一段直行期间完成),HOLD保持跨过转弯,
*等主流程通知(进入第二段直行)后才下落,保证下落发生在第二段直行期间,
*重心变化不会干扰转弯
*/
void lift_sequence() {
    lift.move_voltage(LIFT_UP_SIGN * LIFT_MV);
    pros::delay(LIFT_UP_MS);//等1.2秒抬升完成,再进入HOLD刹车模式
    lift.move_voltage(0);   //HOLD刹车模式接管,原地保持

    uint32_t t0 = pros::millis();
   
    while ((g_lift_down_at == 0 || pros::millis() < g_lift_down_at) &&
           pros::millis() - t0 < (uint32_t)LIFT_DOWN_WAIT_CAP_MS) {
        pros::delay(10);
    }

    lift.move_voltage(-LIFT_UP_SIGN * LIFT_MV);
    pros::delay(LIFT_DOWN_MS);
    lift.move_voltage(0);
}

/**
*@brief 开局设定刹车模式 
*@see set_brake_mode() *@brief 传入*@param pros::E_MOTOR_BRAKE_BRAKE 让小车直接停下
*/
void autonomous() {
    left_drive.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    right_drive.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    
    g_lift_down_at = 0;//清掉上一轮残留,否则抬升任务会立刻下落
    pros::Task lift_task([] { lift_sequence(); }, "lift_seq");
    /*
    个人认为这里很重要:
    由于我们所使用的V5是单核,在一瞬间只能执行一条指令流(不等于只能跑一个任务),但是
    由于我们在autonomous()中调用了pros::Task lift_task([] { lift_sequence(); }, "lift_seq");
    这就意味着lift_sequence()这个函数会在一个独立的任务中运行,就是在另一个栈上执行
    而cpu每1ms会产生一次中断,装逼一点的说就是tick,每次tick都会看情况切换任务(这个"情况"很有说法)
    每次中断,cpu就会先进行调度评估,然后如何可以的话(什么情况才算"可以")立即停下手中的任务,去调度器执行代码
    */
    drive_straight(DRIVE_MV, STRAIGHT_MS, STRAIGHT_P);                        // 直行1.5秒(同时抬起)
    turn_left(TURN_DEG, TURN_P, TURN_MIN_MV, TURN_TOL_DEG, TURN_TIMEOUT_MS);  // 原地左转90度
    g_lift_down_at = pros::millis() + 200;//进入第二段直行0.2秒后下落,1.2秒落完,仍在本段内
    drive_straight(DRIVE_MV, STRAIGHT_MS, STRAIGHT_P);                        // 直行1.5秒(同时下落)

    lift_task.join();//等抬升时序跑完,避免任务残留到opcontrol跟遥控抢电机
}

/**
*@see get_digital_new_press() *@brief 用于检测手柄上的某个按键是否按下
*/
void opcontrol() {
    while (true) {
        //验轮测试:按住"上"键,六个驱动电机同符号正转;哪个轮子往后转就翻转它端口的符号
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
            left_drive.move_voltage(4000);
            right_drive.move_voltage(4000);
        } else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
            autonomous();  // 无场控时按B原地跑一遍完整自动赛,约4~5秒,期间阻塞其他按键
        } else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            drive_straight(6000, 800, 4.0);  // 按A走0.8秒
        } else {
            left_drive.move_voltage(0);
            right_drive.move_voltage(0);
        }
        //抬升遥控:按住L1抬起、按住L2下落,用于实车确认LIFT_UP_SIGN和双边方向是否一致
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            lift.move_voltage(LIFT_UP_SIGN * LIFT_MV);
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            lift.move_voltage(-LIFT_UP_SIGN * LIFT_MV);
        } else {
            lift.move_voltage(0);
        }
        pros::delay(20);
    }
}
