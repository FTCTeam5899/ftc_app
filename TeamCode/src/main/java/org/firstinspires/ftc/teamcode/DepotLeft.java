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
        double times = 0;
        double tTime = 0;
        double angle = getAngle();
        //  Wait until start
        waitForStart();
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE_VIOLET);
        //locks servo in place
        mServo.setPosition(0.33);
        //moves forward, turns left, then slowly
        //turns until is aligned with cube
        move(300,0.4,0.4);
        move(600, -0.3, 0.3);
        while(!goldDetector.getAligned() && !isStopRequested()){
            if(goldDetector.isFound()){
                x =  goldDetector.getXPosition();
                if(x >= 320) {
                    move(50, 0.3, -0.3);
                    tTime += 50;
                }
                else{
                    move(50, -0.3, 0.3);
                    tTime += 50;
                }
            }
            else {
                move(50, 0.3, -0.3);
                tTime += 50;
            }
        }
        goldDetector.alignSize = 640.0;
        x =  goldDetector.getXPosition();
        times = 3100;
        //drives toward cube until it can not be found any longer
        while(x<630.0 && x>10.0 && goldDetector.isFound() && times > 0 && !isStopRequested()){
            //resets detector
            goldDetector.alignSize = 100.0;
            while(!goldDetector.getAligned() && times > 0 && !isStopRequested()){
               x =  goldDetector.getXPosition();
               lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
               if(x >= 320) {
                   move(50, 0.3, -0.3);
               }
               else{
                   move(50, -0.3, 0.3);
               }
               times -= 50;
               telemetry.addData("Location", goldDetector.getXPosition());
               telemetry.addData("Found", goldDetector.isFound());
               telemetry.addData("time", times);
               telemetry.update();
            }
            while(goldDetector.getAligned() && times > 0 && !isStopRequested()){
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                goldDetector.alignSize = 400.0;
               move(50,0.5,0.5);
               times -= 50;
               telemetry.addData("Location",goldDetector.getXPosition());
               telemetry.addData("Found", goldDetector.isFound());
               telemetry.addData("time", times);
               telemetry.update();
            }
            goldDetector.alignSize = 640.0;
            telemetry.addData("Location",goldDetector.getXPosition());
            telemetry.addData("Found", goldDetector.isFound());
            telemetry.addData("time", times);
            telemetry.update();
        }
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_RAINBOW_PALETTE);
        //moves forward
        move(500, 0.5, 0.5);

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

        turnTo(40,.25);

        pause(500);

        resetAngle();
        if(tTime < 700){
            moveStraight(2200, 0.2);
        }
        else if(tTime >= 700 && tTime < 1200){
            moveStraight(2600, 0.2);
        }
        else if(tTime >= 1200){
            moveStraight(3000, 0.2);
        }
        else{
            telemetry.addData("Aligner", "Error" + tTime);
        }


        //%%%%%%%%
        //%%%%%%%%
        //%%%%%%%%
        //%%%%%%%%


        resetAngle();
        moveStraight(500, -0.2);

        pause(200);

        resetAngle();
        turnTo(80,.25);

        resetAngle();
        moveStraight(4000, -0.2);

        resetAngle();
        moveStraight(500, 0.3);
        mServo.setPosition(0.68);
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
