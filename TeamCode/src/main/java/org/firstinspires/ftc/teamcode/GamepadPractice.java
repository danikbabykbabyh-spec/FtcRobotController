package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.MotorTest;

@TeleOp
public class GamepadPractice extends OpMode {

    MotorTest bench = new MotorTest();

    @Override
    public void init() {
        bench.init(hardwareMap);
    }

    @Override
    public void loop() {
        bench.setMotorSpeedCruise(gamepad1.left_stick_y / 2);
    }
}