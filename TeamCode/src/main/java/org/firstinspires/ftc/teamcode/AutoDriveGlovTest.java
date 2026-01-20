package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="AutoDriveGlovTest") // Имя для выбора на Станции Водителя
public class AutoDriveGlovTest extends LinearOpMode {
    private DcMotor FLDrive = null;
    private DcMotor FRDrive = null;
    private DcMotor BLDrive = null;
    private DcMotor BRDrive = null;
    private ElapsedTime runtime = new ElapsedTime();

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

    @Override
    public void runOpMode() {
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

        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        waitForStart(); // Ожидание старта

        // --- НАЧАЛО МАРШРУТА ---
        if (opModeIsActive()) {

            // Шаг 1: Движение к точке (-34.27, 32.93). Расстояние ~14 дюймов.
            telemetry.addData("Step", "1: Driving forward 14 inches");
            telemetry.update();
            encoderDrive(DRIVE_SPEED, 14, 14, 4.0);

            // Шаг 2: Поворот примерно на 35 градусов вправо.
            double turnInches1 = INCHES_FOR_90_DEG_TURN * (35.0 / 90.0);
            telemetry.addData("Step", "2: Turning right ~35 degrees");
            telemetry.update();
            encoderDrive(TURN_SPEED, turnInches1, -turnInches1, 3.0);

            // Шаг 3: Движение к точке (-22.54, 30.66). Расстояние ~12 дюймов.
            telemetry.addData("Step", "3: Driving forward 12 inches");
            telemetry.update();
            encoderDrive(DRIVE_SPEED, 12, 12, 4.0);

            // Шаг 4: Поворот примерно на 43 градуса влево.
            double turnInches2 = INCHES_FOR_90_DEG_TURN * (43.0 / 90.0);
            telemetry.addData("Step", "4: Turning left ~43 degrees");
            telemetry.update();
            encoderDrive(TURN_SPEED, -turnInches2, turnInches2, 3.0);

            // Шаг 5: Движение к конечной точке (-13.97, 36.08). Расстояние ~10 дюймов.
            telemetry.addData("Step", "5: Driving forward 10 inches");
            telemetry.update();
            encoderDrive(DRIVE_SPEED, 10, 10, 3.0);

            telemetry.addData("Path", "Complete");
            telemetry.update();
        }
    }

    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) {
        if (opModeIsActive()) {
            int newLeftTarget = FLDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            int newRightTarget = FRDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            int newBackLeftTarget = BLDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            int newBackRightTarget = BRDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);

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

            while (opModeIsActive() && (runtime.seconds() < timeoutS) &&
                   (FLDrive.isBusy() && FRDrive.isBusy() && BLDrive.isBusy() && BRDrive.isBusy())) {
                // Ожидаем завершения движения, ничего не делаем в цикле
            }

            // Остановка моторов
            FLDrive.setPower(0);
            FRDrive.setPower(0);
            BLDrive.setPower(0);
            BRDrive.setPower(0);

            // Возврат к обычному режиму
            FLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(100); // Небольшая пауза для стабильности
        }
    }
}
