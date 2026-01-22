package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.MechanismMotor;
import org.firstinspires.ftc.teamcode.mechanisms.MechanismServo;

@TeleOp
public class GlovTeleOp extends OpMode {

    MechanismMotor drive = new MechanismMotor();
    MechanismServo servo = new MechanismServo();

    double throttle, spin, speed, speed1, angle, shootSpeed;

    @Override
    public void init()
    {
        drive.init(hardwareMap);
        servo.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        throttle = gamepad1.right_stick_x;
        spin = -gamepad1.left_stick_y;
        speed = gamepad1.right_trigger;
        speed1 = -gamepad1.left_trigger;
        angle = gamepad2.left_stick_x;
        shootSpeed = gamepad2.right_stick_y;

        drive.drive(throttle, spin);
        drive.shoot(shootSpeed);
        drive.intake(-1);
        if (speed > 0) {
            drive.sideDrive(speed);
        }
        else if (speed1 < 0) {
            drive.sideDrive(speed1);
        }
        servo.setServoPos(gamepad2.right_trigger);
        if (gamepad2.right_trigger == 0) {
            servo.setServoPos(0);
        }
    }
}