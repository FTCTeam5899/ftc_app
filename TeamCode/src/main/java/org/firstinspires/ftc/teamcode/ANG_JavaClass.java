package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp
    public class ANG_JavaClass extends LinearOpMode {
    hardwareMap robot = new hardwareMap();
    
        @Override
        public void runOpMode() {
            robot.init(hardwareMap);
            telemetry.addData("Status", "Initialized");
            telemetry.update();
            robot.digitalTouch.setMode(DigitalChannel.Mode.INPUT);
            // Wait for the game to start (driver presses PLAY)
            waitForStart();
            // run until the end of the match (driver presses STOP)
            double tgtpower = 0;
            while (opModeIsActive()) {

                telemetry.addData("Status", "Started");
                telemetry.update();
                tgtpower = -this.gamepad1.left_stick_y;
                robot.Andy.setPower(tgtpower);
                if(gamepad1.y) {
                    robot.Seral.setPosition(0);
                }
                else if(gamepad1.x || gamepad1.b){
                    robot.Seral.setPosition(0.5);
                }
                else if(gamepad1.a){
                    robot.Seral.setPosition(1);
                }
                if(robot.digitalTouch.getState() == false){
                    telemetry.addData("Button", "PRESSED");
                    telemetry.update();
                } else{
                    telemetry.addData("Button","NOT PRESSED");
                    telemetry.update();
                }
                telemetry.addData("Distance (cm)",robot.Carly.getDistance(DistanceUnit.CM));
                telemetry.update();
            }
        }
    }




































