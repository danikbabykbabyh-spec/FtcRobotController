package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.MechanismMotor;
import org.firstinspires.ftc.teamcode.mechanisms.MechanismServo;

@TeleOp
public class GlovTeleOp extends OpMode {

    MechanismMotor drive = new MechanismMotor();
    MechanismServo servo = new MechanismServo();

    double throttle, spin, shootSpeed, intakeSpeed;

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
        shootSpeed = gamepad2.right_stick_y;
        intakeSpeed = gamepad2.left_stick_y;

        drive.drive(throttle, spin);
        drive.shoot(shootSpeed);
        drive.intake(intakeSpeed);
    }
}