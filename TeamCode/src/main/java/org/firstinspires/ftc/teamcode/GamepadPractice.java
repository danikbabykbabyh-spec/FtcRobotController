package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class GamepadPractice extends OpMode {

    @Override
    public void init() {

    }

    @Override
    public void loop() {
        double diffRearTriggers = gamepad1.right_trigger + gamepad1.left_trigger;

        telemetry.addData("x", gamepad1.right_stick_x);
        telemetry.addData("y", gamepad1.right_stick_y);
        telemetry.addData("b", gamepad1.b);
        telemetry.addData("Difference of rear triggers", diffRearTriggers);
    }
}