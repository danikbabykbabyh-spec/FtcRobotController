package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoTest {
    private Servo servoPos;
    private Servo servoPos2;

    public void init(HardwareMap hwMap) {
        servoPos = hwMap.get(Servo.class, "servoPos");
        servoPos2 = hwMap.get(Servo.class, "servoPos2");
    }

    public void setServoPos(double angle) {
        servoPos.setPosition(angle);
        servoPos2.setPosition(angle);
    }
}