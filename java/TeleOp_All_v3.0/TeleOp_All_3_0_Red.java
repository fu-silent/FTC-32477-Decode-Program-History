package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * TeleOp v3.0.3 Red (修复IMU resetYaw兼容性问题)
 * * 职能划分：
 * - Driver 1 (主驾驶/全能):
 * - 移动(LStick), 旋转(RStick), 模式切换(X/Y), 自动瞄准(A=45°/B=61°),
 * - **航向角重置 (Dpad Up=0°, Left=90°, Down=180°, Right=-90°)**
 * - 拾取(RT/LB), 装填(LT)
 * - Driver 2 (副驾驶/瞄准):
 * - 发射(RT), 装填反转(LB), **反转500RPM(RB)**, 精确转向(RStick*0.5), 设定转速(Dpad)
 * * 此代码基于 2025-2026 FTC 赛季 'Decode' 的麦克纳姆轮底盘和发射器（Shooter）子系统设计。
 */
@TeleOp(name = "TeleOp_All_3_0_Red", group = "TeleOp")
public class TeleOp_All_3_0_Red extends LinearOpMode {

    // ========== 1. 常数定义 (Constants) ==========
    class Constants {
        // 机器人物理尺寸 (略)
        final double WHEEL_DIAMETER_MM = 101.6; // 麦克纳姆轮直径 (4英寸 ≈ 101.6mm)
        final double TRACK_WIDTH_MM = 390.0;    // 轮距/轴距 (影响运动学，此处未用于驱动计算)

        // 硬件映射名称 (务必与 Robot Configuration 匹配)
        final String CHASSIS_MOTOR_LF = "lf";   // 左前电机
        final String CHASSIS_MOTOR_RF = "rf";   // 右前电机
        final String CHASSIS_MOTOR_LB = "lb";   // 左后电机
        final String CHASSIS_MOTOR_RB = "rb";   // 右后电机
        final String MOTOR_INTAKE = "intake";   // 拾取电机
        final String MOTOR_LOAD = "load";       // 装填电机
        final String MOTOR_SHOOTER_1 = "s1";    // 发射器电机 1 (Ex型，支持速度控制)
        final String MOTOR_SHOOTER_2 = "s2";    // 发射器电机 2 (Ex型，支持速度控制)
        final String SENSOR_IMU = "imu";        // 惯性测量单元

        // 操纵杆死区 (防止摇杆轻微漂移导致意外运动)
        final double JOYSTICK_DEADZONE = 0.1;

        // 发射机 PIDF & 物理参数
        final double SHOOTER_TICKS = 28;    // 电机每转的编码器计数值
        // PIDF 系数通常需要针对特定电机进行调优
        final double SHOOTER_F = 14;        // F (Feedforward) 系数：预估达到目标速度所需的基础功率
        final double SHOOTER_P = 250;       // P (Proportional) 系数
        final double SHOOTER_I = 0;         // I (Integral) 系数
        final double SHOOTER_D = 100;       // D (Derivative) 系数

        // 转速预设 (RPM) - 针对不同的发射距离或角度预设
        final int RPM_LONG = 1950;          // 长距离/高转速
        final int RPM_SIDE = 1400;          // 侧边/中距离
        final int RPM_BASE = 1200;          // 近距离/基础
        final int RPM_TOP = 1625;           // 顶部/中高转速
        final int RPM_TOLERANCE = 50;       // 达标判断的容忍度 (实际 RPM 偏离目标 RPM ±50 视为达标)
        final int RPM_REVERSE = 500;        // 反转目标转速 500 RPM (用于清理卡住的物品)

        // 电机功率 (固定功率，非速度控制)
        final double POWER_INTAKE = 0.9;
        final double POWER_LOAD = 0.75;

        // 自动转向参数 (针对场上的固定目标角度，如塔或目标区域)
        final double TURN_POWER = 0.8;                     // 自动转向的最大输出功率
        final double TURN_TARGET_TRIANGLE_TOP = 45.0;      // D1 A 键: 三角顶部瞄准 (目标场向角度)
        final double TURN_TARGET_FAR = 61.0;               // D1 B 键: 超远瞄准 (目标 61度场向角度)

        // 交互参数
        final int RUMBLE_MS = 200;                  // 手柄震动时长 (毫秒)
        final double D2_TURN_COEFFICIENT = 0.5;     // Driver 2 精细瞄准系数 (将转向摇杆输入减半，实现慢速精确瞄准)

        final String VERSION = "v3.0.3 Red";        // 版本更新
    }

    // ========== 2. 底盘驱动系统 (ChassisDrive) ==========
    class ChassisDrive {
        DcMotor fl, fr, bl, br;
        Navigation nav;
        boolean isNonLinear = true;     // 摇杆输入是否使用非线性（平方）映射
        boolean isFieldCentric = true;  // 是否使用场地中心模式（无头模式）

        ChassisDrive(DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br, Navigation nav) {
            this.fl = fl; this.fr = fr; this.bl = bl; this.br = br; this.nav = nav;
        }

        void init() {
            // 设置电机方向 (通常根据机械安装和麦轮配置确定)
            fl.setDirection(DcMotor.Direction.FORWARD);
            fr.setDirection(DcMotor.Direction.REVERSE);
            bl.setDirection(DcMotor.Direction.REVERSE); // 注意：此处左后电机与左前电机方向不同，符合FTC常见的麦轮配置
            br.setDirection(DcMotor.Direction.FORWARD);

            // 零功率行为：刹车 (BRAKE) 比自由滑行 (FLOAT) 更利于精确停止
            fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        void toggleMode() { isNonLinear = !isNonLinear; }        // 切换摇杆输入模式
        void toggleCentric() { isFieldCentric = !isFieldCentric; } // 切换无头模式/机器人模式

        String getModeStr() {
            String centric = isFieldCentric ? "地面 Field" : "车辆 Robot"; // 场地中心 vs 机器人自身中心
            String linear = isNonLinear ? "非线性 Curve" : "  线性 Linear"; // 非线性（精细控制） vs 线性
            return centric + " / " + linear;
        }

        /**
         * 更新底盘电机功率
         * @param drive 前/后输入
         * @param strafe 左/右输入 (平移)
         * @param turn 旋转输入
         * @param isAutoTurn 是否处于自动转向模式
         * @param autoPower 自动转向的功率（由导航系统计算）
         */
        void update(double drive, double strafe, double turn, boolean isAutoTurn, double autoPower) {
            // 摇杆死区处理
            if (Math.abs(drive) < constants.JOYSTICK_DEADZONE) drive = 0;
            if (Math.abs(strafe) < constants.JOYSTICK_DEADZONE) strafe = 0;

            if (isAutoTurn) {
                // 如果是自动转向，则覆盖手动转向输入
                turn = autoPower;
                // 注意：自动转向时仍允许手动移动/平移
            } else {
                // 手动控制时：
                if (Math.abs(turn) < constants.JOYSTICK_DEADZONE) turn = 0;
                if (isNonLinear) {
                    // 非线性映射 (输入值平方，保留符号)：使小输入更精细，大输入仍能达到最大速度
                    drive = Math.copySign(drive * drive, drive);
                    strafe = Math.copySign(strafe * strafe, strafe);
                    turn = Math.copySign(turn * turn, turn);
                }
            }

            // 无头模式坐标转换 (Field Centric Drive)
            // 将摇杆指令 (drive, strafe) 从场地坐标系转换到机器人坐标系
            if (isFieldCentric) {
                double botHeading = Math.toRadians(nav.getHeading()); // 当前机器人朝向 (弧度)
                // 旋转矩阵乘法：将 (strafe, drive) 逆时针旋转 -botHeading
                double rotX = strafe * Math.cos(-botHeading) - drive * Math.sin(-botHeading);
                double rotY = strafe * Math.sin(-botHeading) + drive * Math.cos(-botHeading);
                strafe = rotX;
                drive = rotY;
                // turn 旋转分量保持不变
            }

            // 麦克纳姆轮驱动运动学 (Mecanum Kinematics)
            // 计算四个轮子的所需功率 (速度)
            double fL = drive + strafe + turn; // 左前
            double fR = drive - strafe - turn; // 右前
            double bL = drive - strafe + turn; // 左后
            double bR = drive + strafe - turn; // 右后

            // 归一化 (Normalization)：防止功率超过 ±1.0
            double max = Math.max(Math.abs(fL), Math.max(Math.abs(fR), Math.max(Math.abs(bL), Math.abs(bR))));
            if (max > 1.0) { fL /= max; fR /= max; bL /= max; bR /= max; }

            // 设置电机功率
            fl.setPower(fL); fr.setPower(fR); bl.setPower(bL); br.setPower(bR);
        }

        void stop() { fl.setPower(0); fr.setPower(0); bl.setPower(0); br.setPower(0); }
    }

    // ========== 3. 子系统 (Subsystems) ==========
    class Subsystems {
        DcMotor intake, load;       // 拾取和装填电机 (通常是定速/定功率)
        DcMotorEx s1, s2;           // 发射器电机 (ExtendenDcMotor，支持速度控制)
        int targetRPM = 0;          // D-Pad 设置的目标 RPM (仅用于正转达标判断和显示)
        boolean isShooting = false; // 意图进行正向发射 (用于 isAtSpeed 和震动逻辑)

        int currentRunningRPM = 0;  // 实际指令的 RPM 绝对值 (用于遥测显示)
        int currentDirection = 0;   // 存储当前指令的方向 (1=FWD, -1=REV, 0=STOP)

        Subsystems(DcMotor intake, DcMotor load, DcMotorEx s1, DcMotorEx s2) {
            this.intake = intake; this.load = load; this.s1 = s1; this.s2 = s2;
        }

        void init() {
            intake.setDirection(DcMotor.Direction.FORWARD);
            load.setDirection(DcMotor.Direction.REVERSE);
            s1.setDirection(DcMotor.Direction.FORWARD);
            // S2 设置为 REVERSE，确保 S1 和 S2 在收到相同符号的指令时，物理方向一致
            s2.setDirection(DcMotor.Direction.REVERSE);

            intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            load.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // 配置 Shooter 电机的 PIDF 系数，设置为 RUN_USING_ENCODER 模式
            PIDFCoefficients pidf = new PIDFCoefficients(constants.SHOOTER_P, constants.SHOOTER_I, constants.SHOOTER_D, constants.SHOOTER_F);
            s1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
            s2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
            // 确保电机使用编码器进行速度控制
            s1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            s2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        void setTargetRPM(int rpm) {
            this.targetRPM = rpm;
            // setActualVelocity 在主循环中处理
        }

        void setShooting(boolean active) {
            this.isShooting = active;
            // setActualVelocity 在主循环中处理
        }

        /**
         * 设置 Shooter 电机的实际转速和方向
         * @param rpm 目标转速的绝对值 (RPM)
         * @param direction 1=正转, -1=反转, 0=停止
         */
        void setActualVelocity(int rpm, int direction) {
            this.currentRunningRPM = rpm;
            this.currentDirection = direction;

            // 将 RPM 转换为 FTC Velocity 单位 (Ticks/sec)
            // Ticks/sec = RPM * Ticks/Rev / 60 sec/min
            double v_magnitude = rpm * constants.SHOOTER_TICKS / 60.0;

            double v_s1 = v_magnitude * direction;
            // FIX: 修正 S2 速度计算。由于 S2 在 init 中设置为 REVERSE，
            // 必须给它和 S1 相同的符号才能获得相同的物理转动方向。
            double v_s2 = v_magnitude * direction;

            s1.setVelocity(v_s1);
            s2.setVelocity(v_s2);
        }

        // 获取 S1 电机转速 (RPM)
        // RPM = (Velocity / Ticks/Rev) * 60
        double getShooter1RPM() { return (s1.getVelocity() / constants.SHOOTER_TICKS) * 60.0; }
        // 获取 S2 电机转速 (RPM)
        double getShooter2RPM() { return (s2.getVelocity() / constants.SHOOTER_TICKS) * 60.0; }


        /**
         * 达标判断：只针对正向发射 (isShooting = true) 且目标转速 (targetRPM)
         * @return S1 和 S2 的实际转速是否都达到目标转速的容忍范围内
         */
        boolean isAtSpeed() {
            // 只有在正向发射意图时才进行达标检测 (负数转速永远不会达标)
            if (!isShooting || targetRPM <= 0) return false;

            // 检查 S1 (S1 期望是正转)
            boolean s1_ready = Math.abs(getShooter1RPM() - targetRPM) < constants.RPM_TOLERANCE;
            // 检查 S2 (S2 期望是正转)
            boolean s2_ready = Math.abs(getShooter2RPM() - targetRPM) < constants.RPM_TOLERANCE;

            // 必须 S1 AND S2 都达标
            return s1_ready && s2_ready;
        }

        /**
         * 运行拾取电机
         * @param dir 1=正转 (拾取), -1=反转 (弹出/清理), 0=停止
         */
        void runIntake(int dir) {
            intake.setPower(dir * constants.POWER_INTAKE);
        }

        /**
         * 运行装填电机
         * @param dir 1=正转 (装填), -1=反转 (弹出/清理), 0=停止
         */
        void runLoad(int dir) {
            load.setPower(dir * constants.POWER_LOAD);
        }

        void stopAll() {
            runIntake(0);
            runLoad(0);
            setShooting(false);
            setActualVelocity(0, 0); // Stop shooter motors, resets currentRunningRPM and currentDirection
        }

        String getIntakeStr() { return intake.getPower() > 0 ? "正转" : (intake.getPower() < 0 ? "反转" : "停止"); }
        String getLoadStr() { return load.getPower() > 0 ? "正转" : (load.getPower() < 0 ? "反转" : "停止"); }

        // 使用记录的 currentDirection 来准确报告 Shooter 状态
        String getShooterStatusStr() {
            if (currentDirection == 0) return "停止";
            // 正转意图，使用 D-Pad 设定转速
            if (currentDirection == 1) return "发射";
            // 反转意图，固定 500 RPM
            if (currentDirection == -1) return "反转";
            return "待机"; // 理论上不会出现
        }
    }

    // ========== 4. 导航系统 (Navigation) ==========
    class Navigation {
        IMU imu;
        boolean autoTurning = false;            // 是否正在执行自动转向
        double currentAutoTurnTarget = 0.0;     // 自动转向的目标场向角度
        double yawOffset = 0.0;                 // 新增：软件层面的偏航角补偿/偏移量

        Navigation(IMU imu) { this.imu = imu; }

        void init() {
            // IMU 初始化：设置安装方向
            imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                    RevHubOrientationOnRobot.UsbFacingDirection.UP)));
            // 调用无参数的 resetYaw()，将 IMU 内部读数重置为 0
            imu.resetYaw();
            yawOffset = 0.0; // 软件偏移量也设为 0
        }

        // 修改：获取当前机器人的航向角 (Yaw)，并应用软件偏移量
        double getHeading() {
            // 读取 IMU 的当前原始 Yaw 读数
            double currentYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            // 返回 IMU 读数 + 偏移量，并归一化
            return normalize(currentYaw + yawOffset);
        }

        /**
         * 重置航向角到指定目标 (使用软件偏移量补偿)
         * @param targetHeading 目标场向角度 (以 degrees 为单位)
         */
        void resetTo(double targetHeading) {
            // 获取 IMU 当前的原始读数 (不带 yawOffset)
            double currentImuYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            // 计算新的 yawOffset:
            // New_yawOffset = 目标角度 - 原始读数
            yawOffset = targetHeading - currentImuYaw;

            // 确保偏移量也在 [-180, 180) 范围内
            yawOffset = normalize(yawOffset);

            autoTurning = false; // 重置航向角时应停止自动转向
        }

        /**
         * 启动自动转向
         * @param targetHeading 目标场向角度 (以 degrees 为单位)
         */
        void startAutoTurn(double targetHeading) {
            currentAutoTurnTarget = targetHeading;
            autoTurning = true;
        }

        /**
         * 计算自动转向所需的旋转功率 (简单的比例 P 控制)
         * @return 转向功率，如果在容忍范围内则返回 0
         */
        double getAutoTurnPower() {
            if (!autoTurning) return 0;
            double error = normalize(getHeading() - currentAutoTurnTarget); // 计算归一化后的角度误差 (-180° 到 180°)
            if (Math.abs(error) < 2.0) { // 如果误差小于 2.0 度，则认为目标达成
                autoTurning = false;
                return 0;
            }
            // P 控制：功率 = 误差 * 比例系数 (0.05)，并限制在最大功率范围内
            return Range.clip(error * 0.05, -constants.TURN_POWER, constants.TURN_POWER);
        }

        /**
         * 将角度归一化到 [-180, 180) 度范围内
         */
        double normalize(double d) {
            while (d >= 180) d -= 360;
            while (d < -180) d += 360;
            return d;
        }
    }

    // ========== 5. 输入控制 (InputHandler) ==========
    class InputHandler {
        // 按钮状态记录 (用于实现按钮的“按下”触发，而非“按住”触发)
        boolean lastY=false, lastX=false, lastOpt=false;
        boolean lastRight=false, lastLeft=false, lastUp=false, lastDown=false; // D2 RPM D-Pad 跟踪变量
        boolean lastA_P1=false, lastB_P1=false;

        // Driver 1 D-Pad 状态跟踪 (用于航向角重置)
        boolean lastRight_P1=false, lastLeft_P1=false, lastUp_P1=false, lastDown_P1=false;

        boolean d2OverrideActive = false; // 记录 D2 是否正在接管转向，用于 D1 的震动反馈

        // --- Driver 1: 移动 ---
        double getDriveY() { return -gamepad1.left_stick_y; } // Y 轴反转 (向上为正)
        double getDriveX() { return gamepad1.left_stick_x; }

        // 转向控制：D2 优先级 0.5x (精细瞄准)
        double getTurnComposite() {
            double p2 = gamepad2.right_stick_x;
            double p1 = gamepad1.right_stick_x;

            if (Math.abs(p2) > constants.JOYSTICK_DEADZONE) {
                // Driver 2 摇杆输入高于死区，D2 接管转向
                if (!d2OverrideActive) {
                    gamepad1.rumble(constants.RUMBLE_MS); // D2 接管时，给 D1 手柄震动提示
                    d2OverrideActive = true;
                }
                return p2 * constants.D2_TURN_COEFFICIENT; // 返回 D2 的减速后转向输入
            } else {
                d2OverrideActive = false;
                return p1; // 返回 D1 的全速转向输入
            }
        }

        // --- Driver 1: 拾取模块 (RT/LB) ---
        int getIntakeDir() {
            if (gamepad1.right_trigger > 0.1) return 1; // D1 RT: 正转 (拾取)
            if (gamepad1.left_bumper) return -1;        // D1 LB: 反转
            return 0;
        }

        // --- 装填模块 (D1 LT / D2 LB) ---
        int getLoadDir() {
            if (gamepad1.left_trigger > 0.1) return 1; // D1 LT: 正转 (装填/发射前推进)
            if (gamepad2.left_bumper) return -1;      // D2 LB: 反转 (清理卡住的物品)
            return 0;
        }

        // --- 发射模块 (D2 RT) ---
        boolean isShoot() { return gamepad2.right_trigger > 0.1; } // D2 RT: 正转 (发射)

        // --- 反转模块 (D2 RB) ---
        boolean isShooterReverse() { return gamepad2.right_bumper; } // D2 RB: 反转 (清理发射器)

        // --- Driver 1: 自动瞄准 (A/B) ---
        // 检查 A 键是否被“按下” (从 false 变为 true)
        boolean isAimTriangleTopPressed() {
            boolean c=gamepad1.a;
            boolean r=c&&!lastA_P1;
            lastA_P1=c;
            return r;
        }

        // 检查 B 键是否被“按下”
        boolean isAimFarPressed() {
            boolean c=gamepad1.b;
            boolean r=c&&!lastB_P1;
            lastB_P1=c;
            return r;
        }

        // --- Driver 1: 配置键 ---
        boolean isModePressed() { boolean c=gamepad1.y; boolean r=c&&!lastY; lastY=c; return r; } // Y 键：切换非线性/线性模式
        boolean isCentricPressed() { boolean c=gamepad1.x; boolean r=c&&!lastX; lastX=c; return r; } // X 键：切换场地/机器人中心模式

        // --- Driver 1: 航向角重置 (D-Pad) ---
        // 重置为 0 度 (D-Pad Up)
        boolean isReset0Pressed() {
            boolean c=gamepad1.dpad_up;
            boolean r=c&&!lastUp_P1;
            lastUp_P1=c;
            return r;
        }
        // 重置为 90 度 (D-Pad Left)
        boolean isReset90Pressed() {
            boolean c=gamepad1.dpad_left;
            boolean r=c&&!lastLeft_P1;
            lastLeft_P1=c;
            return r;
        }
        // 重置为 180 度 (D-Pad Down)
        boolean isReset180Pressed() {
            boolean c=gamepad1.dpad_down;
            boolean r=c&&!lastDown_P1;
            lastDown_P1=c;
            return r;
        }
        // 重置为 -90 度 (D-Pad Right)
        boolean isResetNeg90Pressed() {
            boolean c=gamepad1.dpad_right;
            boolean r=c&&!lastRight_P1;
            lastRight_P1=c;
            return r;
        }

        // --- Driver 2: RPM 设置 (D-Pad) ---
        /**
         * 检查 D2 的 D-Pad 输入，设置新的目标 RPM
         * @param current 当前的目标 RPM
         * @return 新的目标 RPM
         */
        int checkRPM(int current) {
            int next = current;
            if (dpad(gamepad2.dpad_right, 1)) next = constants.RPM_LONG;     // 右键：长距离
            else if (dpad(gamepad2.dpad_left, 2)) next = constants.RPM_SIDE; // 左键：侧边
            else if (dpad(gamepad2.dpad_down, 3)) next = constants.RPM_BASE; // 下键：基础
            else if (dpad(gamepad2.dpad_up, 4)) next = constants.RPM_TOP;    // 上键：顶部
            return next;
        }

        /**
         * D-Pad 按钮的按下触发逻辑 (用于 D2 的 RPM 设置)
         * @param curr 按钮当前状态
         * @param id 按钮 ID (1=Right, 2=Left, 3=Down, 4=Up)
         */
        boolean dpad(boolean curr, int id) {
            boolean trig = false;
            // 使用 last 变量记录上一次状态，实现“按下”触发 (用于 D2 的 Dpad)
            if (id==1) { trig = curr && !lastRight; lastRight = curr; }
            else if (id==2) { trig = curr && !lastLeft; lastLeft = curr; }
            else if (id==3) { trig = curr && !lastDown; lastDown = curr; }
            else if (id==4) { trig = curr && !lastUp; lastUp = curr; }
            return trig;
        }

        void rumbleDriver2() { gamepad2.rumble(constants.RUMBLE_MS); } // D2 手柄震动
    }

    // ========== 6. 主程序 (Main Loop) ==========
    Constants constants;
    ChassisDrive chassis;
    Subsystems subsys;
    Navigation nav;
    InputHandler input;
    ElapsedTime runtime;
    boolean rpmReadyVibrationTriggered = false; // RPM 达标震动是否已触发

    @Override
    public void runOpMode() {
        constants = new Constants();
        runtime = new ElapsedTime();

        // 硬件映射和初始化
        try {
            DcMotor fl = hardwareMap.get(DcMotor.class, constants.CHASSIS_MOTOR_LF);
            DcMotor fr = hardwareMap.get(DcMotor.class, constants.CHASSIS_MOTOR_RF);
            DcMotor bl = hardwareMap.get(DcMotor.class, constants.CHASSIS_MOTOR_LB);
            DcMotor br = hardwareMap.get(DcMotor.class, constants.CHASSIS_MOTOR_RB);
            DcMotor intake = hardwareMap.get(DcMotor.class, constants.MOTOR_INTAKE);
            DcMotor load = hardwareMap.get(DcMotor.class, constants.MOTOR_LOAD);
            DcMotorEx s1 = hardwareMap.get(DcMotorEx.class, constants.MOTOR_SHOOTER_1);
            DcMotorEx s2 = hardwareMap.get(DcMotorEx.class, constants.MOTOR_SHOOTER_2);
            IMU imu = hardwareMap.get(IMU.class, constants.SENSOR_IMU);

            nav = new Navigation(imu);
            chassis = new ChassisDrive(fl, fr, bl, br, nav);
            subsys = new Subsystems(intake, load, s1, s2);
            input = new InputHandler();

            nav.init();
            chassis.init();
            subsys.init();
        } catch (Exception e) {
            telemetry.addData("Init Failed", e.getMessage());
            telemetry.update();
            return; // 初始化失败，退出 OpMode
        }

        telemetry.addData("Status", constants.VERSION + " Dual Driver Ready");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // 主循环
        while (opModeIsActive()) {

            // --- Driver 1: 功能键与自动瞄准 ---
            // 航向角重置 (D-Pad)
            if (input.isReset0Pressed()) {
                nav.resetTo(0.0);
                gamepad1.rumble(constants.RUMBLE_MS); // 震动反馈
            }
            if (input.isReset90Pressed()) {
                nav.resetTo(90.0);
                gamepad1.rumble(constants.RUMBLE_MS);
            }
            if (input.isReset180Pressed()) {
                nav.resetTo(180.0);
                gamepad1.rumble(constants.RUMBLE_MS);
            }
            if (input.isResetNeg90Pressed()) {
                nav.resetTo(-90.0);
                gamepad1.rumble(constants.RUMBLE_MS);
            }

            if (input.isModePressed()) { chassis.toggleMode(); gamepad1.rumble(200); } // Y 键：切换线性/非线性
            if (input.isCentricPressed()) { chassis.toggleCentric(); gamepad1.rumble(200); } // X 键：切换场地中心/机器人中心

            // D1 A 键: 三角顶部自动瞄准
            if (input.isAimTriangleTopPressed()) {
                nav.startAutoTurn(constants.TURN_TARGET_TRIANGLE_TOP);
            }
            // D1 B 键: 超远自动瞄准
            if (input.isAimFarPressed()) {
                nav.startAutoTurn(constants.TURN_TARGET_FAR);
            }


            // --- Driver 2: RPM 调速 ---
            int newRPM = input.checkRPM(subsys.targetRPM);
            if (newRPM != subsys.targetRPM) {
                subsys.setTargetRPM(newRPM); // 更新目标 RPM
                input.rumbleDriver2();       // D2 震动反馈 RPM 已更改
                rpmReadyVibrationTriggered = false; // 重置达标震动触发器
            }

            // --- 武器控制逻辑更新 ---
            boolean isRev = input.isShooterReverse(); // D2 RB (反转)
            boolean isFwd = input.isShoot();          // D2 RT (发射)

            // 1. 设置 Shooter 意图/速度/方向
            int targetVelocityRPM = 0;
            int direction = 0; // 1=Forward, -1=Reverse, 0=Stop

            if (isRev) {
                // RB按下：反转，500 RPM，优先级最高 (用于清理)
                targetVelocityRPM = constants.RPM_REVERSE;
                direction = -1;
                subsys.setShooting(false); // 清除正转意图 (确保 isAtSpeed 返回 false)
            } else if (isFwd) {
                // RT按下：正转，使用D-Pad设置的RPM (用于发射)
                targetVelocityRPM = subsys.targetRPM;
                direction = 1;
                subsys.setShooting(true); // 设置正转意图
            } else {
                // 都没有按下：停止
                targetVelocityRPM = 0;
                direction = 0;
                subsys.setShooting(false); // 清除正转意图
            }

            // 执行速度设定
            subsys.setActualVelocity(targetVelocityRPM, direction);

            // RPM 达标震动反馈 (D2) - 只有在正转达标时才需要震动反馈
            if (subsys.isShooting && subsys.isAtSpeed()) {
                if (!rpmReadyVibrationTriggered) {
                    gamepad2.rumble(300); // 震动通知 D2 发射器已达标，可以装填/发射
                    rpmReadyVibrationTriggered = true;
                }
            } else { // 只要不是正向达标状态 (包括反转和停止) 就重置震动触发
                rpmReadyVibrationTriggered = false;
            }

            // 其他武器控制 (Intake/Load) 保持不变
            subsys.runIntake(input.getIntakeDir()); // D1 RT/LB 控制拾取
            subsys.runLoad(input.getLoadDir());     // D1 LT / D2 LB 控制装填


            // --- 底盘控制 (包含 D2 瞄准接管逻辑) ---
            chassis.update(
                    input.getDriveY(),
                    input.getDriveX(),
                    input.getTurnComposite(), // 包含 D1/D2 转向逻辑
                    nav.autoTurning,
                    nav.getAutoTurnPower()
            );

            // ===================================
            // ************ 遥测信息更新 ************
            // ===================================
            telemetry.addData("版本", "%s", constants.VERSION);
            telemetry.addData("运行时间", "%.1f s", runtime.seconds());
            telemetry.addLine();

            telemetry.addData("目标转速/是否达标", "%d RPM / %s",
                    subsys.targetRPM, subsys.isAtSpeed() ? "✔ 已达标" : "✘ 未达标");
            telemetry.addData("S1/S2 转速", "S1: %.0f RPM / S2: %.0f RPM",
                    subsys.getShooter1RPM(), subsys.getShooter2RPM());
            telemetry.addLine();

            telemetry.addData("驱动模式", "%s", chassis.getModeStr());
            telemetry.addData("当前航向", "%.1f °", nav.getHeading());
            telemetry.addData("自动转向", "%s / %.1f °",
                    nav.autoTurning ? "进行中" : "已停止", nav.currentAutoTurnTarget);
            // 新增：显示软件偏航角补偿
            telemetry.addData("IMU 补偿", "%.1f °", nav.yawOffset);
            telemetry.addLine();

            telemetry.addData("拾取状态", "%s", subsys.getIntakeStr());
            telemetry.addData("装填状态", "%s", subsys.getLoadStr());
            telemetry.addData("发射状态", "%s", subsys.getShooterStatusStr());
            telemetry.addLine();

            // 更新 Driver 1 遥测信息 (新增 Dpad 航向角重置说明)
            telemetry.addData("Driver 1", "A=45°大三角区, B=61°超远, RT=拾取, LB=反转拾取, LT=装填");
            telemetry.addData("Dpad Yaw Reset", "Up=0°, Left=90°, Down=180°, Right=-90°");
            telemetry.addData("Driver 2", "RT=发射, RB=发射反转500RPM, LB=装填反转, Dpad=预设RPM");

            telemetry.update();
        }

        // 结束时停止所有电机
        chassis.stop();
        subsys.stopAll();
    }
}