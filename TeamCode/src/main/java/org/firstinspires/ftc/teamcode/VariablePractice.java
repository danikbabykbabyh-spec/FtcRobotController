package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class VariablePractice extends OpMode {
    @Override
    public void init() {
        int robotHeight = 67;
        double robotSpeed = 33.45;
        boolean clawOpened = false;
        String name = "Tamos IT";
        int motorAngle = 90;

        telemetry.addData("name", name);
        telemetry.addData("Speed", robotSpeed);
        telemetry.addData("Claw status", clawOpened);
        telemetry.addData("Height", robotHeight);
        telemetry.addData("Motor angle", motorAngle);
    }

    @Override
    public void loop() {

    }
}
