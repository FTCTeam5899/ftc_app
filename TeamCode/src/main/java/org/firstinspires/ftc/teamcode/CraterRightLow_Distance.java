package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="CraterRightLow_Sensors", group="Official")

public class CraterRightLow_Distance extends AutoSupplies{
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
        double angle = getAngle();
        double currentDistance = 0;
        while(!isStopRequested()){
            x = goldDetector.getXPosition();
            y = goldDetector.getYPosition();
            telemetry.addData("x",x);
            telemetry.addData("y",y);
            telemetry.update();
        }
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

        turnTo(90, 0.7);
        turnTo(90,0.25);
        resetAngle();
        lift.setTargetPosition(0);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(1);
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE_VIOLET);

        //moves forward, turns left, then slowly turns until is aligned with cube
        move(300, 0.4, 0.4);
        move(800, -0.3, 0.3);
        goldDetector.alignSize = 50.0;//283 419
        telemetry.clear();
        while(!isStopRequested()) {
            x = goldDetector.getXPosition();
            y = goldDetector.getYPosition();
            telemetry.addData("x",x);
            telemetry.addData("y",y);
            telemetry.update();
            if (goldDetector.isFound() && y>= 330) {
                telemetry.addData("working",y);
                telemetry.update();
                if (x >= 320 && !goldDetector.getAligned()) {
                    setPower(0.24,-0.24);
                    tTime += 1;
                } else if (x <= 320 && !goldDetector.getAligned()) {
                    setPower(-0.24,0.24);
                    tTime += 1;
                }
                else{
                    break;
                }
            } else {
                setPower(0.3,-0.3);
                tTime += 1;
            }
        }
        telemetry.clear();
        setPower(0,0);
        telemetry.addData("x",x);
        telemetry.addData("y",y);
        goldDetector.alignSize = 640.0;

        telemetry.addData("time", tTime);
        telemetry.update();
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_RAINBOW_PALETTE);
        //moves forward
        move(1000, 0.6, 0.6);
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_RAINBOW_PALETTE);
        //moves backwards
        move(750, -0.5, -0.5);
        //turns toward the depot with 2 statements to ensure accuracy and speed
        turnTo(90,0.5);
        turnTo(90,0.25);
        //determines if the cube was left right or center and moves straight for the allotted time
        moveStraight(1800, 0.5);
        //drives toward wall and turns to face it
        pause(100);
        resetAngle();
        pause(100);
        turnTo(-45,0.5);
        turnTo(-45,0.25);
        //moves to and aligns with the wall
        move(1300, 0.3, 0.3);
        //backs up
        move(400, -0.3, -0.3);
        //turns to face crater and backs into depot
        pause(100);
        resetAngle();
        pause(100);
        turnTo(-87,0.5);
        turnTo(-88,0.25);
        moveStraight(1600, -1.0);
        //drops the team marker, and moves to the crater
        mServo.setPosition(0.68);
        resetAngle();
        /*
        This While loop is used to navigate from the depot back to the crater. It stays in the for
        loop until it drives over the crater wall. It knows it is on the wall when the imu, set to
        measure pitch, senses that the robot is on a upward facing plane. In order to get to the
        wall, we use a light distance sensor to follow along a wall adjusting motor power as needed.
        If the robot is driven so severely off course by some unknown factor such as hitting another
        robot, the gyro sensor will detect that and turn the robot back to a bearing similar to the
        angle at which it started driving to get to the crater.
         */
        telemetry.clear();
        while(getPitch() < 4.0 && getPitch() > -4.0 && !isStopRequested()){
            telemetry.addData("Pitch",getPitch());
            currentDistance = distanceSensorL.getDistance(DistanceUnit.CM);
            if(getAngle() > 25) {
                turnTo(0, 0.5);
            }
            else if(getAngle() < -25){
                turnTo(0,0.5);
            }
            else {
                if (currentDistance < 9) {
                    left = 0.6;
                    right = right * 0.9;
                } else if (currentDistance > 11) {
                    left = left * 0.9;
                    right = 0.6;
                } else {
                    left = 0.6;
                    right = 0.6;
                }
                motorFwdLeft.setPower(left);
                motorFwdRight.setPower(-right);
                motorBackLeft.setPower(-left);
                motorBackRight.setPower(right);
            }
            telemetry.addData("Left", left);
            telemetry.addData("Right", right);
            telemetry.addData("Distance", currentDistance);
            telemetry.update();
        }
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED_ORANGE);

        //  Turn all motors off and sleep
        motorFwdLeft.setPower(0);
        motorFwdRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
        sleep(1000);
    }
}
