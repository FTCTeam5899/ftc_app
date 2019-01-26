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

@Autonomous(name="DepotLeft", group="Official")

public class DepotLeft extends AutoSupplies{


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
        //locks servo on place
        mServo.setPosition(0.33);
        //lowers bot from lander
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_RED);
        lift.setTargetPosition(24250);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(1);
        //backs it off the lander and turns
        while (lift.getCurrentPosition() <= 24150 && !isStopRequested()) {
        }
        pause(200);
        move(500, -0.6, -0.6);
        move(500, -0.2, 0.8);

        turnTo(90, 0.5);
        turnTo(90, 0.25);
        resetAngle();
        lift.setTargetPosition(0);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(1);
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE_VIOLET);

        //moves forward, turns left, then slowly
        //turns until is aligned with cube
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
            if (goldDetector.isFound() && y >= 340) {
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
            moveStraight(1400, 0.5);
            turnTo(83, 0.5);
            turnTo(83, 0.25);
        } else {
            telemetry.addData("center", 0);
            telemetry.update();
            turnTo(10, 0.5);
            turnTo(10, 0.25);
            move(600, 0.5, 0.5);
            turnTo(145, 0.5);
            turnTo(145, 0.25);
            moveStraight(600, -0.5);
            resetAngle();
            turnTo(-20, 0.6);
            turnTo(-22, 0.25);
        }


        resetAngle();
        mServo.setPosition(0.68);
        resetAngle();
        turnTo(-1, 0.6);
        move(2500, 1, 1);
        if (angle >= 20) {
            move(700, 0.5, 0);
        }
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_RED);
        pause(3000);
        goldDetector.alignSize = 100.0;

        //  Turn all motors off and sleep
        motorFwdLeft.setPower(0);
        motorFwdRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
        sleep(1000);

    }

}
