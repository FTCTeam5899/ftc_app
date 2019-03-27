package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name="Aligner Test", group="Tests")
@Disabled
public class NewAligner extends AutoSupplies{
    @Override
    public void runOpMode() {

        //  Establish all hardware and initialize camera
        enableGoldDetector();
        telemetry.addData("Status", "DogeCV 2018.0 - Gold Aligner");

        initForAutonomous();
        double x1 = 0;
        double x2 = 0;
        double x3 = 0;
        double y1 = 0;
        double y2 = 0;
        double y3 = 0;
        double times = 0;
        double tTime = 0;
        double angle = getAngle();
        //  Wait until start
        waitForStart();
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE_VIOLET);
        //moves forward, turns left, then slowly
        //turns until is aligned with cube
        move(300,0.4,0.4);
        turnTo(8, 0.25);
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        x1 = goldDetector.getXPosition();
        y1 = goldDetector.getYPosition();
        pause(3000);
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE_VIOLET);
        resetAngle();
        turnTo(-24, 0.25);
        x2 = goldDetector.getXPosition();
        y2 = goldDetector.getYPosition();
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        pause(3000);
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE_VIOLET);
        resetAngle();
        turnTo(8, 0.25);
        x3 = goldDetector.getXPosition();
        y3 = goldDetector.getYPosition();
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_RED);
        telemetry.addData("x1",x1);
        telemetry.addData("x2",x2);
        telemetry.addData("x3",x3);
        telemetry.addData("y1",y1);
        telemetry.addData("y2",y2);
        telemetry.addData("y3",y3);
        telemetry.update();
        sleep(10000);
    }
}
