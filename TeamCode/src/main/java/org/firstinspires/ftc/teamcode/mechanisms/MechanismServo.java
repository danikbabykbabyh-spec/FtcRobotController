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
}