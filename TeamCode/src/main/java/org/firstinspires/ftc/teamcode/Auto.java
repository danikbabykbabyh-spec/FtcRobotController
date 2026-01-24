package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name = "Auto: Coordinates (Encoders + IMU)", group = "Auto")
public class Auto extends LinearOpMode {
    private static final String MOTOR_LF = "motorL";
    private static final String MOTOR_RF = "motorR";
    private static final String MOTOR_LB = "motorBL";
    private static final String MOTOR_RB = "motorBR";
    private static final String IMU_NAME  = "imu";
    private static final String SHOOT = "motorShoot";
    private static final String INTAKE = "motorIntake";
    private static final String INTAKE2 = "motorIntake2";






    private static final double TICKS_PER_REV = 6000;
    private static final double WHEEL_RADIUS_IN = 2.95;
    private static final double GEAR_RATIO = 5;
    private static final double LATERAL_MULTIPLIER = 1.10; // подстройка страйфа (обычно 1.0–1.2)

    private static final double KP_XY = 0.06;      // насколько агрессивно едем к точке
    private static final double KP_HEADING = 0.02; // насколько агрессивно крутимся
    private static final double MAX_POWER = 0.4;

    private static final double POS_TOL_IN = 1.0;      // 1 дюйм
    private static final double HEADING_TOL_RAD = Math.toRadians(3); // 3 градуса

    private final Pose[] targets = new Pose[] {

            new Pose(-32.98, -37.77, 0.62),

            new Pose(-16.70, -37.59, 90.00),

        new Pose(-16.70,  24.71, 140.53),

        new Pose(-25.63,  32.06, 270.22),

        new Pose(-25.28, -59.32, 183.00),

        new Pose(-55.38, -60.90, 45.44),

            new Pose(-32.98, -37.77, 0.62),

            new Pose(-16.70, -37.59, 90.00),

            new Pose(-16.70,  24.71, 140.53),

            new Pose(-25.63,  32.06, 270.22),

            new Pose(-25.28, -59.32, 183.00),

            new Pose(-55.38, -60.90, 45.44),

    };

    // ====== Hardware ======
    private DcMotorEx lf, rf, lb, rb, motorS, motorI, motorI2;

    private Servo servoPos, servoPos2;
    private IMU imu;

    // ====== Состояние одометрии ======
    private Pose pose = new Pose(0, 0, 0); // x,y in inches; heading in radians
    private int lastLf, lastRf, lastLb, lastRb;

    @Override
    public void runOpMode() {
        // --- init hardware ---
        lf = hardwareMap.get(DcMotorEx.class, MOTOR_LF);
        rf = hardwareMap.get(DcMotorEx.class, MOTOR_RF);
        lb = hardwareMap.get(DcMotorEx.class, MOTOR_LB);
        rb = hardwareMap.get(DcMotorEx.class, MOTOR_RB);
        motorS = hardwareMap.get(DcMotorEx.class, SHOOT);
        motorI = hardwareMap.get(DcMotorEx.class, INTAKE);
        motorI2 = hardwareMap.get(DcMotorEx.class, INTAKE2);
        servoPos = hardwareMap.get(Servo.class, "servoPos");
        servoPos2 = hardwareMap.get(Servo.class, "servoPos2");

        servoPos.setDirection(Servo.Direction.REVERSE);

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

        // Сброс IMU yaw (в новых SDK это важно)
        imu.resetYaw();

        lastLf = lf.getCurrentPosition();
        lastRf = rf.getCurrentPosition();
        lastLb = lb.getCurrentPosition();
        lastRb = rb.getCurrentPosition();

        telemetry.addLine("Ready. Place robot at start, then press Play.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        for (int i = 0; i < targets.length; i++) {
            Pose t = targets[i];

            goToPose(t.x, t.y, Math.toRadians(t.headingDeg), 7.0);

            // ===== ПОСЛЕ ТРЕТЬЕЙ ТОЧКИ =====
            if (i == 2) {
                stopDrive();
                runMotorShoot(1.0, 2.0);
            }

            if (isStopRequested()) break;
        }
        for (int y = 0; y < targets.length; y++) {
            Pose t = targets[y];

            goToPose(t.x, t.y, Math.toRadians(t.headingDeg), 5.0);


            if (y == 5) {
                stopDrive();
                sleep((4000));
            }

            if (isStopRequested()) break;
        }
    }
    private void runMotorShoot(double power, double seconds) {
        motorS.setPower(power);
        motorI.setPower(power);
        motorI2.setPower(power);
        sleep((long)(seconds * 1000));
        motorS.setPower(0);
        motorI.setPower(0);
        motorI2.setPower(0);
    }

    /**
     * Едет к целевой позе (x,y,heading). heading в радианах.
     */
    private void goToPose(double targetX, double targetY, double targetHeadingRad, double timeoutSec) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while (opModeIsActive() && timer.seconds() < timeoutSec) {
            updatePose(); // обновляем pose.x, pose.y, pose.heading

            // Ошибка в координатах поля
            double ex = targetX - pose.x;
            double ey = targetY - pose.y;

            // Ошибка по углу
            double eHeading = angleWrap(targetHeadingRad - pose.heading);

            // Если в допуске — выходим
            if (Math.hypot(ex, ey) < POS_TOL_IN && Math.abs(eHeading) < HEADING_TOL_RAD) {
                break;
            }

            // Переводим ошибку из координат поля в координаты робота
            // (чтобы “вперёд” означало ехать туда, куда смотрит робот)
            double cos = Math.cos(pose.heading);
            double sin = Math.sin(pose.heading);

            // robotX = ошибка вперёд, robotY = ошибка влево (страйф)
            double robotX =  ex * cos + ey * sin;
            double robotY = -ex * sin + ey * cos;

            // P-контроль (можно заменить на PID)
            double vx = KP_XY * robotX;         // вперёд/назад
            double vy = KP_XY * robotY;         // страйф
            double omega = KP_HEADING * eHeading; // поворот

            // Ограничиваем
            vx = clip(vx, -MAX_POWER, MAX_POWER);
            vy = clip(vy, -MAX_POWER, MAX_POWER);
            omega = clip(omega, -MAX_POWER, MAX_POWER);

            // Mecanum микс
            double pLF = vx + vy + omega;
            double pRF = vx - vy - omega;
            double pLB = vx - vy + omega;
            double pRB = vx + vy - omega;

            // Нормализация (чтобы ни один мотор не вышел за [-1..1])
            double max = Math.max(1.0, Math.max(Math.abs(pLF),
                    Math.max(Math.abs(pRF), Math.max(Math.abs(pLB), Math.abs(pRB)))));

            pLF /= max; pRF /= max; pLB /= max; pRB /= max;

            setDrivePower(pLF, pRF, pLB, pRB);

            telemetry.addData("Target", "x=%.1f y=%.1f h=%.1f°", targetX, targetY, Math.toDegrees(targetHeadingRad));
            telemetry.addData("Pose",   "x=%.1f y=%.1f h=%.1f°", pose.x, pose.y, Math.toDegrees(pose.heading));
            telemetry.addData("Err",    "dx=%.1f dy=%.1f dh=%.1f°", ex, ey, Math.toDegrees(eHeading));
            telemetry.update();
        }

        stopDrive();
    }

    /**
     * Обновляет pose по дельтам энкодеров и heading из IMU.
     * Ось: x = вперёд по полю, y = влево по полю.
     */
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

        // Механум-оценка перемещения в системе робота:
        // forward = среднее всех
        // strafe = комбинация для бокового движения
        double forward = (iLf + iRf + iLb + iRb) / 4.0;
        double strafe  = (-iLf + iRf + iLb - iRb) / 4.0 * LATERAL_MULTIPLIER;

        // Heading берём из IMU (yaw)
        pose.heading = getImuHeadingRad();

        // Поворачиваем (forward/strafe) из системы робота в систему поля
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
        double x;           // inches
        double y;           // inches
        double heading;     // radians (for internal use)
        double headingDeg;  // degrees (for easy targets list)

        Pose(double x, double y, double headingDeg) {
            this.x = x;
            this.y = y;
            this.headingDeg = headingDeg;
            this.heading = Math.toRadians(headingDeg);
        }
    }
}