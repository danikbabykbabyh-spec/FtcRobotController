package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name = "Auto: Simple Forward", group = "Auto")
public class Auto extends LinearOpMode {
    private static final String MOTOR_LF = "motorL";
    private static final String MOTOR_RF = "motorR";
    private static final String MOTOR_LB = "motorBL";
    private static final String MOTOR_RB = "motorBR";
    private static final String IMU_NAME  = "imu";
    private static final String SHOOT = "motorShoot";
    private static final String INTAKE = "motorIntake";
    private static final String INTAKE2 = "motorPush";

    // Константы одометрии
    private static final double TICKS_PER_REV = 6000;
    private static final double WHEEL_RADIUS_IN = 2.95;
    private static final double GEAR_RATIO = 5;

    // PID коэффициенты
    private static final double KP_XY = 0.12;
    private static final double KP_HEADING = 0.04;
    private static final double MAX_POWER = 0.6;

    private static final double POS_TOL_IN = 1.0;
    private static final double HEADING_TOL_RAD = Math.toRadians(3);

    // Простое движение вперёд на 20 дюймов
    private static final double FORWARD_DISTANCE = 20.0; // дюймы

    // Hardware
    private DcMotorEx lf, rf, lb, rb, motorS, motorI, motorI2;
    private IMU imu;

    // Состояние одометрии
    private Pose pose = new Pose(0, 0, 0);
    private int lastLf, lastRf, lastLb, lastRb;

    @Override
    public void runOpMode() {
        initHardware();

        telemetry.addLine("Ready. Robot will drive forward " + FORWARD_DISTANCE + " inches.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        telemetry.addLine("Driving forward...");
        telemetry.update();

        // Едем вперёд на заданное расстояние
        driveForward(FORWARD_DISTANCE, 1);

        telemetry.addLine("Auto Complete!");
        telemetry.update();
    }

    private void initHardware() {
        lf = hardwareMap.get(DcMotorEx.class, MOTOR_LF);
        rf = hardwareMap.get(DcMotorEx.class, MOTOR_RF);
        lb = hardwareMap.get(DcMotorEx.class, MOTOR_LB);
        rb = hardwareMap.get(DcMotorEx.class, MOTOR_RB);
        motorS = hardwareMap.get(DcMotorEx.class, SHOOT);
        motorI = hardwareMap.get(DcMotorEx.class, INTAKE);
        motorI2 = hardwareMap.get(DcMotorEx.class, INTAKE2);

        motorS.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(IMU.class, IMU_NAME);

        rf.setDirection(DcMotorEx.Direction.REVERSE);
        rb.setDirection(DcMotorEx.Direction.REVERSE);



        lf.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        lf.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rf.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        lb.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        imu.resetYaw();

        lastLf = lf.getCurrentPosition();
        lastRf = rf.getCurrentPosition();
        lastLb = lb.getCurrentPosition();
        lastRb = rb.getCurrentPosition();
    }

    /**
     * Едет вперёд на заданное расстояние
     */
    private void driveForward(double distanceInches, double timeoutSec) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        // Запоминаем начальную позицию
        double startX = pose.x;
        double startY = pose.y;
        double targetHeading = pose.heading; // держим текущий угол

        while (opModeIsActive() && timer.seconds() < timeoutSec) {
            updatePose();

            // Вычисляем пройденное расстояние
            double distanceTraveled = Math.hypot(pose.x - startX, pose.y - startY);
            double distanceRemaining = distanceInches - distanceTraveled;

            // Ошибка по углу (чтобы ехать прямо)
            double eHeading = angleWrap(targetHeading - pose.heading);

            // Если доехали — выходим
            if (distanceRemaining < POS_TOL_IN) {
                break;
            }

            // P-регулятор для движения вперёд
            double vx = KP_XY * distanceRemaining;
            vx = clip(vx, -MAX_POWER, MAX_POWER);

            // Коррекция угла, чтобы ехать прямо
            double omega = KP_HEADING * eHeading;
            omega = clip(omega, -MAX_POWER * 0.5, MAX_POWER * 0.5);

            // Mecanum микс (только вперёд + коррекция угла)
            double pLF = vx + omega;
            double pRF = vx - omega;
            double pLB = vx + omega;
            double pRB = vx - omega;

            // Нормализация
            double max = Math.max(1.0, Math.max(Math.abs(pLF),
                    Math.max(Math.abs(pRF), Math.max(Math.abs(pLB), Math.abs(pRB)))));

            pLF /= max; pRF /= max; pLB /= max; pRB /= max;

            setDrivePower(pLF, pRF, pLB, pRB);

            telemetry.addData("Target Distance", "%.1f in", distanceInches);
            telemetry.addData("Traveled", "%.1f in", distanceTraveled);
            telemetry.addData("Remaining", "%.1f in", distanceRemaining);
            telemetry.addData("Heading", "%.1f°", Math.toDegrees(pose.heading));
            telemetry.update();
        }

        stopDrive();
    }

    private void updatePose() {
        int curLf = lf.getCurrentPosition();
        int curRf = rf.getCurrentPosition();
        int curLb = lb.getCurrentPosition();
        int curRb = rb.getCurrentPosition();

        int dLf = curLf - lastLf;
        int dRf = curRf - lastRf;
        int dLb = curLb - lastLb;
        int dRb = curRb - lastRb;

        lastLf = curLf;
        lastRf = curRf;
        lastLb = curLb;
        lastRb = curRb;

        // Тики -> дюймы
        double iLf = ticksToInches(dLf);
        double iRf = ticksToInches(dRf);
        double iLb = ticksToInches(dLb);
        double iRb = ticksToInches(dRb);

        // Mecanum одометрия
        double forward = (iLf + iRf + iLb + iRb) / 4.0;
        double strafe  = (-iLf + iRf + iLb - iRb) / 4.0;

        pose.heading = getImuHeadingRad();

        // Преобразование в глобальные координаты
        double cos = Math.cos(pose.heading);
        double sin = Math.sin(pose.heading);

        double dX = forward * cos - strafe * sin;
        double dY = forward * sin + strafe * cos;

        pose.x += dX;
        pose.y += dY;
    }

    private double getImuHeadingRad() {
        YawPitchRollAngles ypr = imu.getRobotYawPitchRollAngles();
        double yawDeg = ypr.getYaw(AngleUnit.DEGREES);
        return Math.toRadians(yawDeg);
    }

    private double ticksToInches(int ticks) {
        double revolutions = ticks / TICKS_PER_REV;
        double wheelDistance = 2.0 * Math.PI * WHEEL_RADIUS_IN * revolutions;
        return wheelDistance / GEAR_RATIO;
    }

    private void setDrivePower(double pLF, double pRF, double pLB, double pRB) {
        lf.setPower(pLF);
        rf.setPower(pRF);
        lb.setPower(pLB);
        rb.setPower(pRB);
    }

    private void stopDrive() {
        setDrivePower(0, 0, 0, 0);
    }

    private static double clip(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private static double angleWrap(double rad) {
        while (rad > Math.PI) rad -= 2.0 * Math.PI;
        while (rad < -Math.PI) rad += 2.0 * Math.PI;
        return rad;
    }

    private static class Pose {
        double x;
        double y;
        double heading;

        Pose(double x, double y, double heading) {
            this.x = x;
            this.y = y;
            this.heading = heading;
        }
    }
}