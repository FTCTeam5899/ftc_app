package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="Just_Land", group="Official")

public class JustLand extends AutoSupplies{
    @Override
    public void runOpMode() {

        //  Establish all hardware and initialize camera
        enableGoldDetector();
        telemetry.addData("Status", "DogeCV 2018.0 - Gold Aligner");

        initForAutonomous();
        double x = 0;
        double y = 0;
        double tTime = 0;
        double left = 0.6;
        double right = 0.6;
        double first = 0.6; // was 0.5
        double second = 0.35; // was 0.25
        double angle = getAngle();
        double currentDistance = 0;
        boolean done = false;
        //  Wait until start
        waitForStart();
        //locks servo on place
        mServo.setPosition(0.33);
        //lowers bot from lander
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_RED);
        lift.setTargetPosition(18250);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(1);
        //backs it off the lander and turns
        while(lift.getCurrentPosition() <=18150 && !isStopRequested()){}
        pause(50);
        resetPitch();
        pause(200);
        move(400, -0.6, -0.6);
        move(500, -0.2, 0.8);

        turnToS(90, 0.7);
        turnToS(90,0.3);
        resetAngle();
        lift.setTargetPosition(0);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(1);

        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED_ORANGE);
        //  Turn all motors off and sleep
        motorFwdLeft.setPower(0);
        motorFwdRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
        sleep(10000);
    }
}
