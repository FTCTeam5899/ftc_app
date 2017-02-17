package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

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

    private UltrasonicSensor ultraL = null;
    private UltrasonicSensor ultraR = null;
    private OpticalDistanceSensor odsSensor;
    private ColorSensor colorSenLeft;
    private ColorSensor colorSenRight;

    //other variables

    float mSpd = .2f;
    float trnSpd = .3f;
    double distance = .3;
    double uLeft;
    double uRight;

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

    public void waiting(long m){
        runtime.reset();
        while ((runtime.milliseconds() < m) && !super.isStopRequested())
        {
            // Wait
        }
    }

    public void stopMot(){
        leftMotors(0);
        rightMotors(0);
    }

    public void setUltra(){
        double tempL = ultraL.getUltrasonicLevel();
        double tempR = ultraR.getUltrasonicLevel();

        if(uLeft != 0){
            uLeft = tempL;
        }
        if(uRight != 0){
            uRight = tempR;
        }

        /*if(Math.abs(uLeft-tempL) <= 8){
            uLeft = tempL;
        }
        if(Math.abs(uRight-tempR) <= 8){
            uRight = tempR;
        }*/
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
        ultraL = hardwareMap.ultrasonicSensor.get("ultraL");
        ultraR = hardwareMap.ultrasonicSensor.get("ultraR");
        odsSensor = hardwareMap.opticalDistanceSensor.get("odsSensor");
        colorSenLeft = hardwareMap.colorSensor.get("colorSenL");
        colorSenRight = hardwareMap.colorSensor.get("colorSenR");


        // eg: Set the drive motor directions:
        // Reverse the motor that runs backwards when connected directly to the battery
        // leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        leftMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        leftMotor2.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        odsSensor.enableLed(true);

        uLeft = ultraL.getUltrasonicLevel();
        uRight = ultraR.getUltrasonicLevel();

        //Reminder set servos

        while (!super.isStarted()) {
            setUltra();
            telemetry.addData("ultraL: ", uLeft);
            telemetry.addData("ultraR: ", uRight);
            telemetry.update();
        }

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

            while (odsSensor.getLightDetected() < .3 && (!super.isStopRequested())) {
                leftMotors(mSpd);
                rightMotors(mSpd);
                telemetry.addData("ODS Light Detected: ", odsSensor.getLightDetected());
                telemetry.update();
            }
            stopMot(); // at white line

            telemetry.addData("ultraL: ", ultraL.getUltrasonicLevel());
            telemetry.addData("ultraR: ", ultraR.getUltrasonicLevel());
            telemetry.addLine("ultraL.status(): " + ultraL.status());
            telemetry.addLine("ultraR.status(): " + ultraR.status());
            telemetry.update();

            waiting(5000);


            while (!(uRight-uLeft >= 3 && uRight-uLeft <= 5)){
                if((uRight-uLeft > 5)) {
                    leftMotors(trnSpd);
                    rightMotors(-trnSpd);
                }
                else if(uRight-uLeft < 3){
                    leftMotors(-trnSpd);
                    rightMotors(trnSpd);
                }
                setUltra();
                telemetry.addData("ultraL: ", uLeft);
                telemetry.addData("ultraR: ", uRight);
                telemetry.update();
            }

            while (ultraL.getUltrasonicLevel() > 8 && (!super.isStopRequested())) {
                leftMotors(mSpd);
                rightMotors(mSpd);
                setUltra();
                telemetry.addData("ultraL: ", uLeft);
                telemetry.addData("ultraR: ", uRight);
                telemetry.update();
            }
            stopMot(); // close to wall

            telemetry.addData("ultraL: ", ultraL.getUltrasonicLevel());
            telemetry.addData("ultraR: ", ultraR.getUltrasonicLevel());
            telemetry.addLine("ultraL.status(): " + ultraL.status());
            telemetry.addLine("ultraR.status(): " + ultraR.status());
            telemetry.update();

            waiting(5000);

            do{
                leftMotors(trnSpd);
                rightMotors(-trnSpd);
                setUltra();
                telemetry.addData("ultraL: ", uLeft);
                telemetry.addData("ultraR: ", uRight);
                telemetry.update();
            }while(uLeft < uRight && (!super.isStopRequested()));

            stopMot();
            waiting(10000);
        stop();

                while(ultraL.getUltrasonicLevel() != distance && (!super.isStopRequested())){
                    leftMotors(-trnSpd);
                    rightMotors(trnSpd);
                    waiting(2000);

                    leftMotors(mSpd);
                    rightMotors(mSpd);
                    waiting(2000);

                    leftMotors(trnSpd);
                    rightMotors(-trnSpd);
                    telemetry.addData("ultraL: ", ultraL.getUltrasonicLevel());
                    telemetry.update();
                    stopMot();
                    waiting(2000);
                }
            while (odsSensor.getLightDetected() < .3 && (!super.isStopRequested())) {
                leftMotors(-mSpd);
                rightMotors(-mSpd);
                telemetry.addData("ODS Light Detected: ", odsSensor.getLightDetected());
                telemetry.update();
                stopMot();
                waiting(2000);
            }


    }

}
