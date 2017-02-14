package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by Radmin on 2/6/2017.
 */


@Autonomous(name="AutonomousSense", group="Iterative Opmode")
//@Disabled
public class AutonomousSense extends LinearOpMode{


/* Declare OpMode members. */

    private DcMotor leftMotor = null;
    private DcMotor leftMotor2 = null;
    private DcMotor rightMotor = null;
    private DcMotor rightMotor2 = null;
    private DcMotor capBallLift = null;
    private Servo rServo = null;
    private Servo pServo = null;

    //sensors

    UltrasonicSensor ultra1 = null;
    UltrasonicSensor ultra2 = null;
    OpticalDistanceSensor odsSensor;
    ColorSensor colorSenLeft;
    ColorSensor colorSenRight;

    //other variables

    float mSpd = .3f;
    double distance = .3;

    private ElapsedTime runtime = new ElapsedTime();


    //functions for The Dominator (Robot 2)
    public void leftMotors(float pow){
        leftMotor.setPower(-pow);
        leftMotor2.setPower(leftMotor.getPower());
    }

    public void  rightMotors(float pow){
        rightMotor.setPower(-pow);
        rightMotor2.setPower(rightMotor.getPower());
    }

    @Override
    public void runOpMode() {
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */

        leftMotor = hardwareMap.dcMotor.get("left_drive");
        leftMotor2 = hardwareMap.dcMotor.get("left_drive2");
        rightMotor = hardwareMap.dcMotor.get("right_drive");
        rightMotor2 = hardwareMap.dcMotor.get("right_drive2");
        capBallLift = hardwareMap.dcMotor.get("capLift");
        rServo = hardwareMap.servo.get("forkDrop");
        pServo = hardwareMap.servo.get("pushServo");
        ultra1 = hardwareMap.ultrasonicSensor.get("ultra1");
        ultra2 = hardwareMap.ultrasonicSensor.get("ultra2");
        odsSensor = hardwareMap.opticalDistanceSensor.get("odsSensor");
        colorSenLeft = hardwareMap.colorSensor.get("colorSen1");
        colorSenRight = hardwareMap.colorSensor.get("colorSen2");


        // eg: Set the drive motor directions:
        // Reverse the motor that runs backwards when connected directly to the battery
        // leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        leftMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        leftMotor2.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        odsSensor.enableLed(true);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (!super.isStopRequested()) {
            while (odsSensor.getLightDetected() < .3) {
                leftMotors(mSpd);
                rightMotors(mSpd);
                telemetry.addData("ODS Light Detectected: ", odsSensor.getLightDetected());
                telemetry.update();
            }

            while(ultra1.getUltrasonicLevel() != ultra2.getUltrasonicLevel()){
                leftMotors(mSpd);
                rightMotors(-mSpd);
                telemetry.addData("ultra1: ", ultra1.getUltrasonicLevel());
                telemetry.addData("ultra2; ", ultra2.getUltrasonicLevel());
                telemetry.update();
            }
                while(ultra1.getUltrasonicLevel() != distance){
                    leftMotors(-mSpd);
                    rightMotors(mSpd);

                    leftMotors(mSpd);
                    rightMotors(mSpd);

                    leftMotors(mSpd);
                    rightMotors(-mSpd);
                    telemetry.addData("ultra1: ", ultra1.getUltrasonicLevel());
                    telemetry.update();
                }
            while (odsSensor.getLightDetected() < .5) {
                leftMotors(-mSpd);
                rightMotors(-mSpd);
                telemetry.addData("ODS Light Detectected: ", odsSensor.getLightDetected());
                telemetry.update();
            }
         }


    }

}
