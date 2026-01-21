package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.MotorTest;
import org.firstinspires.ftc.teamcode.mechanisms.ServoTest;

@TeleOp
public class GamepadPractice extends OpMode {

    MotorTest drive = new MotorTest();
    ServoTest servo = new ServoTest();

    double throttle, spin, speed, speed1, angle, shootSpeed;

    @Override
    public void init()
    {
        drive.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        throttle = gamepad1.right_stick_x;
        spin = -gamepad1.left_stick_y;
        speed = gamepad1.right_trigger;
        speed1 = -gamepad1.left_trigger;
        angle = gamepad2.left_stick_x;
        shootSpeed = gamepad2.right_trigger;

        drive.drive(throttle, spin);
        drive.shoot(shootSpeed);
        drive.intake(1);
        servo.setServoPos(angle);
    }
}