package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Disabled

public class AutoDriveGlov {
    private DcMotor FLDrive = null;
    private DcMotor FRDrive = null;
    private DcMotor BLDrive = null;
    private DcMotor BRDrive = null;
    private ElapsedTime runtime = new ElapsedTime();
    private IMU imu = null;

    // --- ВАЖНЫЕ КОНСТАНТЫ --- (Их нужно настроить под вашего робота)
    static final double COUNTS_PER_MOTOR = 1440; // Зависит от ваших моторов
    static final double DRIVE_GEAR_REDUCTION = 1.0; // Передаточное число редуктора
    static final double WHEEL_DIAMETER_INCHES = 2.95; // Диаметр колес в дюймах
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    static final double DRIVE_SPEED = 0.3; // Скорость движения вперед
    static final double TURN_SPEED = 0.25; // Скорость поворота

    // --- КАЛИБРОВКА ПОВОРОТА ---
    // Это значение нужно подобрать экспериментально. Оно определяет, сколько
    // "дюймов" нужно проехать колесам с одной стороны, чтобы робот повернулся на 90 градусов.
    final double INCHES_FOR_90_DEG_TURN = 12.5;


    public void init(HardwareMap hardwareMap) {
        // Инициализация моторов
        FLDrive = hardwareMap.get(DcMotor.class, "motorL");
        FRDrive = hardwareMap.get(DcMotor.class, "motorR");
        BLDrive = hardwareMap.get(DcMotor.class, "motorBL");
        BRDrive = hardwareMap.get(DcMotor.class, "motorBR");

        FLDrive.setDirection(DcMotor.Direction.REVERSE);
        BLDrive.setDirection(DcMotor.Direction.REVERSE);
        FRDrive.setDirection(DcMotor.Direction.FORWARD);
        BRDrive.setDirection(DcMotor.Direction.FORWARD);

        FLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


    }

    public void loop() {
        double robotRotate = AngleUnit.normalizeRadians
                (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
    }
    public void rotateDrive(double speed, double rotate, double robotRotate) {








    }
    public void driveRobot() {

    }
}
