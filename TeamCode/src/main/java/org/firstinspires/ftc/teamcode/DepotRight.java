package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="DepotRight", group="Official")

public class DepotRight extends AutoSupplies{


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
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE_VIOLET);
        //locks servo in place
        mServo.setPosition(0.35);
        //moves forward, turns left, then slowly
        //turns until is aligned with cube
        move(300,0.4,0.4);
        move(600, -0.3, 0.3);
        goldDetector.alignSize = 50.0;
        while(!goldDetector.getAligned() && !isStopRequested()) {
            if (goldDetector.isFound() && y <= 380) {
                x = goldDetector.getXPosition();
                y = goldDetector.getYPosition();
                if (x >= 320) {
                    move(50, 0.2, -0.2);
                    tTime += 50;
                } else if (x <= 320) {
                    move(50, -0.2, 0.2);
                    tTime += 50;
                }
            } else {
                move(50, 0.3, -0.3);
                tTime += 50;
            }
        }
        sleep(5000);
        telemetry.addData("x",x);
        telemetry.addData("y",y);
        goldDetector.alignSize = 640.0;

        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_RAINBOW_PALETTE);
        //moves forward
        move(1000, 0.5, 0.5);
        sleep(10000);
        //determines if the cube was left right or center and turns toward the crater
        if(tTime < 700){
            telemetry.addData("Founds", "left" + tTime);
            move(600, 0.4, 0);
        }
        else if(tTime >= 700 && tTime < 1200){
            telemetry.addData("Founds", "Center" + tTime);
        }
        else if(tTime >= 1200){
            telemetry.addData("Founds", "Right" + tTime);
            move(600, 0, 0.4);
        }
        else{
            telemetry.addData("Founds", "Error" + tTime);
        }
        //moves forward
        move(900, 0.5, 0.5);
        angle = getAngle();
        //resets detector
        telemetry.addData("Location", goldDetector.getXPosition());
        telemetry.addData("Found", goldDetector.isFound());
        telemetry.addData("Angle", angle);
        telemetry.update();



        sleep(500);

        turnTo(-40,.25);

        pause(500);

        resetAngle();
        if(tTime < 700){
            moveStraight(3000, 0.2);
        }
        else if(tTime >= 700 && tTime < 1200){
            moveStraight(2600, 0.2);
        }
        else if(tTime >= 1200){
            moveStraight(2200, 0.2);
        }
        else{
            telemetry.addData("Aligner", "Error" + tTime);
        }

        resetAngle();
        moveStraight(500, -0.2);

        pause(200);

        resetAngle();
        turnTo(-80,.25);

        resetAngle();
        moveStraight(4000, -0.2);

        resetAngle();
        moveStraight(500, 0.3);
        mServo.setPosition(0.7);
        pause(200);
        resetAngle();
        moveStraight(4000, 0.4);
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED_ORANGE);
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
