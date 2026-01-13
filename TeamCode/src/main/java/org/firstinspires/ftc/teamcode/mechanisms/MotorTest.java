package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MotorTest {
    private DcMotor motorR;
    private DcMotor motorL;
    private DcMotor motorBR;
    private DcMotor motorBL;
    public void init(HardwareMap hwMap) {
        motorR = hwMap.get(DcMotor.class, "motorR");
        motorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorL = hwMap.get(DcMotor.class, "motorL");
        motorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR = hwMap.get(DcMotor.class, "motorBR");
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL = hwMap.get(DcMotor.class, "motorBL");
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void setMotorSpeedCruise(double speedCruise) {
        motorR.setPower(speedCruise *= -1);
        motorL.setPower(speedCruise);
        motorBR.setPower(speedCruise *= -1);
        motorBL.setPower(speedCruise);
    }
}
