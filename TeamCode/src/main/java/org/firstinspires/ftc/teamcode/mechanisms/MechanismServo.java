package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class MechanismServo {
    private Servo servoPos;
    private Servo servoPos2;

    public void init(HardwareMap hwMap) {
        servoPos = hwMap.get(Servo.class, "servoPos");
        servoPos2 = hwMap.get(Servo.class, "servoPos2");
        servoPos.setDirection(Servo.Direction.REVERSE);
    }

    public void setServoPos(double angle) {
        servoPos.setPosition(angle / 2);
        servoPos2.setPosition(angle / 2);
    }
}