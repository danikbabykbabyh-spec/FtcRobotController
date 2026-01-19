package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.MotorTest;

@TeleOp
public class GamepadPractice extends OpMode {

    MotorTest drive = new MotorTest();
    double throttle, spin, speed, speed1;

    @Override
    public void init() {
        drive.init(hardwareMap);
    }

    @Override
    public void loop() {
        throttle = gamepad1.right_stick_x;
        spin = -gamepad1.left_stick_y;
        speed = gamepad1.right_trigger;
        speed1 = -gamepad1.left_trigger;

        drive.drive(throttle, spin);
    }
}