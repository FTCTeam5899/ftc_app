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
        double times = 0;
        double tTime = 0;
        double angle = getAngle();
        //  Wait until start
        waitForStart();
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE_VIOLET);
        //locks servo in place
        mServo.setPosition(0.35);
        //moves forward, turns left, then slowly
        //turns until is aligned with cube
        move(300, 0.4, 0.4);
        move(600, -0.3, 0.3);
        while (!goldDetector.getAligned() && !isStopRequested()) {
            if (goldDetector.isFound()) {
                x = goldDetector.getXPosition();
                if (x >= 320) {
                    move(50, 0.3, -0.3);
                    tTime += 50;
                } else {
                    move(50, -0.3, 0.3);
                    tTime += 50;
                }
            } else {
                move(50, 0.3, -0.3);
                tTime += 50;
            }
        }
        goldDetector.alignSize = 640.0;
        x = goldDetector.getXPosition();
        times = 3100;
        //drives toward cube until it can not be found any longer
        while (x < 630.0 && x > 10.0 && goldDetector.isFound() && times > 0 && !isStopRequested()) {
            //resets detector
            goldDetector.alignSize = 100.0;
            while (!goldDetector.getAligned() && times > 0 && !isStopRequested()) {
                x = goldDetector.getXPosition();
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
                if (x >= 320) {
                    move(50, 0.3, -0.3);
                } else {
                    move(50, -0.3, 0.3);
                }
                times -= 50;
                telemetry.addData("Location", goldDetector.getXPosition());
                telemetry.addData("Found", goldDetector.isFound());
                telemetry.addData("time", times);
                telemetry.update();
            }
            while (goldDetector.getAligned() && times > 0 && !isStopRequested()) {
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                goldDetector.alignSize = 400.0;
                move(50, 0.5, 0.5);
                times -= 50;
                telemetry.addData("Location", goldDetector.getXPosition());
                telemetry.addData("Found", goldDetector.isFound());
                telemetry.addData("time", times);
                telemetry.update();
            }
            goldDetector.alignSize = 640.0;
            telemetry.addData("Location", goldDetector.getXPosition());
            telemetry.addData("Found", goldDetector.isFound());
            telemetry.addData("time", times);
            telemetry.update();
        }
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_RAINBOW_PALETTE);
        //moves forward to knock cube off
        move(500, 0.5, 0.5);
        //moves backwards
        move(600, -0.5, -0.5);
        if(tTime >= 1200){
            move(300, -0.5, -0.5);
        }
        //turns toward the depot
        turnTo(90,0.25);
        //determines if the cube was left right or center and moves straight for the allotted time
        if (tTime < 700) {
            telemetry.addData("Founds", "left" + tTime);
            moveStraight(1100, 0.5);
        } else if (tTime >= 600 && tTime < 1200) {
            telemetry.addData("Founds", "Center" + tTime);
            moveStraight(1325, 0.5);
        } else if (tTime >= 1200) {
            telemetry.addData("Founds", "Right" + tTime);
            moveStraight(1525, 0.5);
        } else {
            telemetry.addData("Founds", "Error" + tTime);
        }
        //drives toward wall and turns to face it
        pause(200);
        resetAngle();
        pause(200);
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
