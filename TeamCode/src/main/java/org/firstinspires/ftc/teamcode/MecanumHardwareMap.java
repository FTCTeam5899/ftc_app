package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class MecanumHardwareMap {
    //Say what exists
    public DcMotor  leftFrontMotor   = null;
    public DcMotor  rightFrontMotor   = null;
    public DcMotor  leftBackMotor   = null;
    public DcMotor  rightBackMotor   = null;

    //Random Garbage
    HardwareMap hwMap =  null;
    private ElapsedTime period  = new ElapsedTime();
    public MecanumHardwareMap(){}
    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        //Name them
       leftFrontMotor  = hwMap.get(DcMotor.class, "leftFrontMotor");

        rightFrontMotor  = hwMap.get(DcMotor.class, "rightFrontMotor");
        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);

        leftBackMotor  = hwMap.get(DcMotor.class, "leftBackMotor");


        rightBackMotor  = hwMap.get(DcMotor.class, "rightBackMotor");
        rightBackMotor.setDirection(DcMotor.Direction.REVERSE);

        // Set to 0
        leftFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightFrontMotor.setPower(0);
        rightBackMotor.setPower(0);


        // encoder?
        //Andy.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Name Servos and set to 0

    }
}