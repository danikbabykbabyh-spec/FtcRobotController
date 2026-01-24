package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MechanismMotor {
    private DcMotor frontRightMotor, backRightMotor, frontLeftMotor, backLeftMotor, shootMotor, intakeMotor, pushMotor;
    public void init(HardwareMap hwMap) {
        frontRightMotor = hwMap.get(DcMotor.class, "motorR");
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftMotor = hwMap.get(DcMotor.class, "motorL");
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor = hwMap.get(DcMotor.class, "motorBR");
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor = hwMap.get(DcMotor.class, "motorBL");
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shootMotor = hwMap.get(DcMotor.class, "motorShoot");
        shootMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shootMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor = hwMap.get(DcMotor.class, "motorIntake");
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pushMotor = hwMap.get(DcMotor.class, "motorPush");
        pushMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pushMotor.setDirection(DcMotor.Direction.FORWARD);


        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    public void drive(double throttle, double spin) {
        double leftPower = throttle + spin;
        double rightPower = throttle - spin;
        double largest = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        if (largest > 1.0) {
            leftPower /= largest;
            rightPower /= largest;
        }

        frontRightMotor.setPower(rightPower);
        frontLeftMotor.setPower(leftPower);
        backRightMotor.setPower(rightPower);
        backLeftMotor.setPower(leftPower);
    }

    public void shoot(double speed) {
        shootMotor.setPower(speed);
    }

    public void intake(double speed) {
        intakeMotor.setPower(speed);
    }

    public void push(double speed) {
        pushMotor.setPower(speed);
    }
}