package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="CraterRightLow", group="Official")

public class CraterRightLow extends AutoSupplies{
    @Override
    public void runOpMode() {

        //  Establish all hardware and initialize camera
        enableGoldDetector();
        telemetry.addData("Status", "DogeCV 2018.0 - Gold Aligner");

        initForAutonomous();
        double x = 0;
        double y = 0;
        double times = 0;
        double tTime = 0;
        double angle = getAngle();
        //  Wait until start
        waitForStart();
        //lowers bot from lander
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_RED);
        lift.setTargetPosition(18500);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(1);
        //backs it off the lander and turns
        while(lift.getCurrentPosition()<=18400 && !isStopRequested()){}
        pause(200);
        move(350, -0.6, -0.6);
        move(500, 0, 0.8);

        turnTo(90, 0.25);
        resetAngle();
        lift.setTargetPosition(0);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(1);
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE_VIOLET);
        //locks servo in place
        mServo.setPosition(0.35);
        //moves forward, turns left, then slowly
        //turns until is aligned with cube
        move(300, 0.4, 0.4);
        move(600, -0.3, 0.3);
        goldDetector.alignSize = 50.0;//283 419
        while(!isStopRequested()) {
            x = goldDetector.getXPosition();
            y = goldDetector.getYPosition();
            if (goldDetector.isFound() && y>= 340) {
                if (x >= 320 && !goldDetector.getAligned()) {
                    setPower(0.24,-0.24);
                    times = getRuntime();
                    //while(times >= getRuntime()-50 && !isStopRequested()){}
                    tTime += 1;
                } else if (x <= 320 && !goldDetector.getAligned()) {
                    setPower(-0.24,0.24);
                    times = getRuntime();
                    //while(times >= getRuntime()-50 && !isStopRequested()){}
                    tTime += 1;
                }
                else{
                    break;
                }
            } else {
                setPower(0.3,-0.3);
                times = getRuntime();
                //while(times >= getRuntime()-50 && !isStopRequested()){}
                tTime += 1;
            }
        }
        setPower(0,0);
        telemetry.addData("x",x);
        telemetry.addData("y",y);
        goldDetector.alignSize = 640.0;

        telemetry.addData("time", tTime);
        telemetry.update();
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_RAINBOW_PALETTE);
        //moves forward
        move(1000, 0.5, 0.5);
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_RAINBOW_PALETTE);
        //moves backwards
        move(750, -0.5, -0.5);
        //turns toward the depot with 2 statements to ensure accuracy and speed
        turnTo(90,0.5);
        turnTo(90,0.25);
        //determines if the cube was left right or center and moves straight for the allotted time
        moveStraight(1525, 0.5);
        //drives toward wall and turns to face it
        pause(200);
        resetAngle();
        pause(200);
        turnTo(-45,0.25);
        //moves to and aligns with the wall
        move(1300, 0.3, 0.3);
        //backs up
        move(600, -0.2, -0.2);
        //turns to face crater and backs into depot
        pause(200);
        resetAngle();
        pause(200);
        turnTo(-90,0.25);
        moveStraight(3800, -0.4);
        //moves forward, drops the team marker, and moves to the crater
        moveStraight(400, 0.5);
        mServo.setPosition(0.7);
        pause(200);
        resetAngle();
        //drives into crater
        moveStraight(5000, 0.7);
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED_ORANGE);



        //  Turn all motors off and sleep
        motorFwdLeft.setPower(0);
        motorFwdRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
        sleep(1000);
    }
}
