package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class MechanismServo {
    private Servo servoPos;
    private Servo servoPos2;

    public void init(HardwareMap hwMap) {
        servoPos = hwMap.get(Servo.class, "servoPos");
        servoPos2 = hwMap.get(Servo.class, "servoPos2");
    }

    public void servo() {
        servoPos.setPosition(1.0);
        servoPos2.setPosition(1.0);
    }
}