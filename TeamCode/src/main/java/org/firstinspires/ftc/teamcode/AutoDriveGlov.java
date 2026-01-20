package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Sequential Auto Drive")
public class AutoDriveGlov extends LinearOpMode {
    private DcMotor FLDrive = null;
    private DcMotor FRDrive = null;
    private DcMotor BLDrive = null;
    private DcMotor BRDrive = null;
    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR = 1440;
    static final double DRIVE_GEAR_REDUCTION = 0.7;
    static final double WHEEL_DIAMETER_INCHES = 2.95;
    static final double COUNTS_PER_INCHES = (COUNTS_PER_MOTOR * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

    @Override
    public void runOpMode() {
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

        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // Шаг 1: Движение вперед на 24 дюйма
            encoderDrive(DRIVE_SPEED, 24, 24, 5.0);  // 5 секунд на выполнение

            // Шаг 2: Поворот направо (примерно на 90 градусов)
            encoderDrive(TURN_SPEED, 12, -12, 4.0);

            // Шаг 3: Движение вперед на 12 дюймов
            encoderDrive(DRIVE_SPEED, 12, 12, 3.0);
        }
    }

    // Метод теперь "блокирующий" - он ждет завершения движения
    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) {
        if (opModeIsActive()) {
            int newLeftTarget = FLDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCHES);
            int newRightTarget = FRDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCHES);
            int newBackLeftTarget = BLDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCHES);
            int newBackRightTarget = BRDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCHES);

            FLDrive.setTargetPosition(newLeftTarget);
            FRDrive.setTargetPosition(newRightTarget);
            BLDrive.setTargetPosition(newBackLeftTarget);
            BRDrive.setTargetPosition(newBackRightTarget);

            FLDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FRDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BLDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BRDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            FLDrive.setPower(Math.abs(speed));
            FRDrive.setPower(Math.abs(speed));
            BLDrive.setPower(Math.abs(speed));
            BRDrive.setPower(Math.abs(speed));

            // Цикл выполняется, пока моторы не достигнут цели или не выйдет время
            while (opModeIsActive() && (runtime.seconds() < timeoutS) &&
                   (FLDrive.isBusy() && FRDrive.isBusy() && BLDrive.isBusy() && BRDrive.isBusy())) {
                // Вывод телеметрии для отладки
                telemetry.addData("Path", "Running to %7d : %7d", newLeftTarget, newRightTarget);
                telemetry.addData("Position", "At %7d : %7d", FLDrive.getCurrentPosition(), FRDrive.getCurrentPosition());
                telemetry.update();
            }

            // Остановка моторов
            FLDrive.setPower(0);
            FRDrive.setPower(0);
            BLDrive.setPower(0);
            BRDrive.setPower(0);

            // Возврат к обычному режиму работы с энкодерами
            FLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250); // Небольшая пауза
        }
    }
}
