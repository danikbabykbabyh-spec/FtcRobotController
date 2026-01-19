package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
@Autonomous
public class AutoDriveGlov extends OpMode {
    private DcMotor FLDrive = null;
    private DcMotor FRDrive = null;
    private DcMotor BLDrive = null;
    private DcMotor BRDrive = null;
    private ElapsedTime runtime = new ElapsedTime();


    static final double COUNTS_PER_MOTOR = 1440;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 2.95;
    static final double COUNTS_PER_INCHES = (COUNTS_PER_MOTOR * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
//    static final double TURN_SPEED = 0.5;


    @Override
    public void init()
    {
        FLDrive = hardwareMap.get(DcMotor.class, "motorL");
        FRDrive = hardwareMap.get(DcMotor.class, "motorR");
        BLDrive = hardwareMap.get(DcMotor.class, "motorBL");
        BRDrive = hardwareMap.get(DcMotor.class, "motorBR");

        FLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        FLDrive.setDirection(DcMotor.Direction.REVERSE);
        BLDrive.setDirection(DcMotor.Direction.REVERSE);
        FRDrive.setDirection(DcMotor.Direction.FORWARD);
        BRDrive.setDirection(DcMotor.Direction.FORWARD);


    }
    @Override
    public void loop()
    {
        encoderDrive(DRIVE_SPEED, 20, 20, 6);
    }
    public void encoderDrive(double speed, double leftInches, double rightInches, double time)
    {
        int newLeftTarget;
        int newRightTarget;

        newLeftTarget = ((FLDrive.getTargetPosition() + BLDrive.getTargetPosition()) / 2) + (int)(leftInches * COUNTS_PER_INCHES);
        newRightTarget = ((FRDrive.getTargetPosition() + BRDrive.getTargetPosition()) / 2) + (int)(rightInches * COUNTS_PER_INCHES);

        FLDrive.setTargetPosition(newLeftTarget);
        BLDrive.setTargetPosition(newLeftTarget);
        FRDrive.setTargetPosition(newRightTarget);
        BRDrive.setTargetPosition(newRightTarget);

        FLDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BLDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FRDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BRDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        runtime.reset();
        FLDrive.setPower(speed);
        BLDrive.setPower(speed);
        FRDrive.setPower(speed);
        BRDrive.setPower(speed);

        if (runtime.seconds() >= time)
        {
            FLDrive.setPower(0);
            BLDrive.setPower(0);
            FRDrive.setPower(0);
            BRDrive.setPower(0);

            FLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}
