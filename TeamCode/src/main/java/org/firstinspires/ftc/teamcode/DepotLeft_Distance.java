package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="DepotLeft_Sensors", group="Official")

public class DepotLeft_Distance extends AutoSupplies{


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
        while (lift.getCurrentPosition() <= 18150 && !isStopRequested()){}
        pause(200);
        resetPitch();
        move(500, -0.6, -0.6);
        move(500, -0.2, 0.8);

        turnTo(90, 0.5);
        turnTo(90, 0.25);
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
        while (!isStopRequested()) {
            x = goldDetector.getXPosition();
            y = goldDetector.getYPosition();
            telemetry.addData("x", x);
            telemetry.addData("y", y);
            telemetry.update();
            if (goldDetector.isFound() && y >= 300) {
                telemetry.addData("working", y);
                telemetry.update();
                if (x >= 320 && !goldDetector.getAligned()) {
                    setPower(0.24, -0.24);
                    tTime += 1;
                } else if (x <= 320 && !goldDetector.getAligned()) {
                    setPower(-0.24, 0.24);
                    tTime += 1;
                } else {
                    break;
                }
            } else {
                setPower(0.3, -0.3);
                tTime += 1;
            }
        }
        angle = getAngle();
        telemetry.clear();
        setPower(0, 0);
        telemetry.addData("x", x);
        telemetry.addData("y", y);
        goldDetector.alignSize = 640.0;

        telemetry.addData("time", tTime);
        telemetry.update();
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_RAINBOW_PALETTE);
        //moves forward
        move(1900, 0.6, 0.6);
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_RAINBOW_PALETTE);
        //resets detector
        telemetry.addData("Location", goldDetector.getXPosition());
        telemetry.addData("Found", goldDetector.isFound());
        telemetry.addData("Angle", angle);
        telemetry.update();


        if (angle >= 20) {
            telemetry.addData("left", 0);
            telemetry.update();
            turnTo(135, 0.5);
            turnTo(135, 0.25);
            moveStraight(1400, -0.5);

        } else if (angle <= -20) {
            telemetry.addData("right", 0);
            telemetry.update();
            turnTo(45, 0.5);
            turnTo(45, 0.25);
            resetAngle();
            moveStraight(1700, 0.5);
            turnTo(92, 0.5);
            turnTo(92, 0.25);
        } else {
            telemetry.addData("center", 0);
            telemetry.update();
            turnTo(70, 0.5);
            turnTo(70, 0.25);
            move(800, 0.5, 0.5);
            turnTo(145, 0.5);
            turnTo(145, 0.25);
            resetAngle();
            turnTo(-10, 0.6);
            turnTo(-11, 0.25);
        }


        resetAngle();
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
            currentDistance = distanceSensorR.getDistance(DistanceUnit.CM);
            if(getAngle() > 25) {
                turnTo(0, 0.5);
            }
            else if(getAngle() < -25){
                turnTo(0,0.5);
            }
            else {
                if (currentDistance > 11) {
                    left = 0.6;
                    right = right * 0.9;
                } else if (currentDistance < 9) {
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
