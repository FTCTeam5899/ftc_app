package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


/**
 * Created by Radmin on 1/9/2017.
 */
@Disabled
public class RobotClass {
    public DcMotor leftMotor = null;
    public DcMotor leftMotor2 = null;
    public DcMotor rightMotor = null;
    public DcMotor rightMotor2 = null;
    public DcMotor catapultMotor = null;
    public DcMotor intake = null;
    public DcMotor capBallLift = null;

    public void init(HardwareMap hardwareMap){
        leftMotor  = hardwareMap.dcMotor.get("left_drive");
        leftMotor2 = hardwareMap.dcMotor.get("left_drive2");
        rightMotor = hardwareMap.dcMotor.get("right_drive");
        rightMotor2 = hardwareMap.dcMotor.get("right_drive2");
        capBallLift = hardwareMap.dcMotor.get("capLift");
        //catapultMotor = hardwareMap.dcMotor.get("cat");
        //intake = hardwareMap.dcMotor.get("intake");

        rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        rightMotor2.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        leftMotor.setPower(0);
        leftMotor2.setPower(0);
        rightMotor.setPower(0);
        rightMotor2.setPower(0);
        capBallLift.setPower(0);

    }

    public void leftMotors(float pow){
        leftMotor2.setPower(-pow);
        leftMotor.setPower(-pow);
    }

    public void  rightMotors(float pow){
        rightMotor2.setPower(-pow);
        rightMotor.setPower(-pow);
    }
}
