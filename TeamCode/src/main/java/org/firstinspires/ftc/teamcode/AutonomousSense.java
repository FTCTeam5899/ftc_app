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

    //motors
    private DcMotor leftMotor = null;
    private DcMotor leftMotor2 = null;
    private DcMotor rightMotor = null;
    private DcMotor rightMotor2 = null;
    private DcMotor capBallLift = null;

    //servos
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
    int[] cLeft = null; //new int[3];
    int[] cRight = null; //new int[3];
    final String COLOR = "Blue";


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
        move(0);
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

    public void move(float pwr){
        leftMotors(pwr);
        rightMotors(pwr);
    }

    public void leftTurn(float pwr){
        leftMotors(pwr);
        rightMotors(-pwr);
    }
    public void leftTurn(float pwr, long t){
        leftMotors(pwr);
        rightMotors(-pwr);
        waiting(t);
    }

    public void rightTurn(float pwr){
        leftMotors(-pwr);
        rightMotors(pwr);
    }
    public void rightTurn(float pwr, long t){
        leftMotors(-pwr);
        rightMotors(pwr);
        waiting(t);
    }

    //method to push the beacons
    /*Color sensor Values when directed at beacon (passive)
        Blue
            Red: 0-2
            Green: 0-1
            Blue: 6-8
        Red
            Red: 4-5
            Green: 0-1
            Blue: 0-1
        */
    public void pushBeacon(String color){
        String leftColor = "";
        String rightColor = "";
        cLeft = new int[] {colorSenLeft.red(), colorSenLeft.green(), colorSenLeft.blue()};
        cRight = new int[] {colorSenRight.red(), colorSenRight.green(), colorSenRight.blue()};

        //find the left color
        if(cLeft[0] <= 2 && cLeft[1] <= 1 && cLeft[2] >= 6){
            leftColor = "BLUE";
        }
        else if (cLeft[0] >= 4 && cLeft[1] <= 1 && cLeft[2] <= 1){
            leftColor = "RED";
        }

        //find the right color
        if(cRight[0] <= 2 && cRight[1] <= 1 && cRight[2] >= 6){
            rightColor = "BLUE";
        }
        else if (cRight[0] >= 4 && cRight[1] <= 1 && cRight[2] <= 1){
            rightColor = "RED";
        }

        //push the correct button on the beacon
        if(leftColor.equalsIgnoreCase(color)){
            //number bigger than .52
            pServo.setPosition(.7);
        }
        else if (rightColor.equalsIgnoreCase(color)){
            //number less than .52
            pServo.setPosition(.3);
        }
    }
;

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

        //set ultrasonic variables
        uLeft = ultraL.getUltrasonicLevel();
        uRight = ultraR.getUltrasonicLevel();

        //set color sensor variables
        colorSenLeft.enableLed(false);
        colorSenRight.enableLed(false);
        cLeft = new int[]{colorSenLeft.red(), colorSenLeft.green(), colorSenLeft.blue()};
        cRight = new int[]{colorSenRight.red(), colorSenRight.green(), colorSenRight.blue()};


        //set servos
        rServo.setPosition(.95);
        pServo.setPosition(.52);

        while (!super.isStarted()) {
            setUltra();
            telemetry.addData("ultraL: ", uLeft);
            telemetry.addData("ultraR: ", uRight);
            telemetry.update();
        }

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //move until the white line is reached
        while (odsSensor.getLightDetected() < .3 && (!super.isStopRequested())) {
            move(mSpd);
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

        //change the angle of the robot so that it can get closer to a wall
        while (!(uRight - uLeft >= 3 && uRight - uLeft <= 5)) {
            if ((uRight - uLeft > 5)) {
                leftTurn(trnSpd);
            } else if (uRight - uLeft < 3) {
                rightTurn(trnSpd);
            }
            setUltra();
            telemetry.addData("ultraL: ", uLeft);
            telemetry.addData("ultraR: ", uRight);
            telemetry.update();
        }

        //move the robot closer to the wall
        while (ultraL.getUltrasonicLevel() > 17 && (!super.isStopRequested())) {
            move(mSpd);
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

        //make the robot parallel to the wall
        //repeat for accuracy
        for (int i = 0; i < 3; i++){
            setUltra();
            while (uLeft < uRight && (!super.isStopRequested())) {
                leftTurn(trnSpd);
                setUltra();
                telemetry.addData("ultraL: ", uLeft);
                telemetry.addData("ultraR: ", uRight);
                telemetry.addData("Iteration: ", i);
                telemetry.update();
            }
            stopMot();
            telemetry.addData("Iteration: ", i);
            telemetry.update();
            waiting(5000);
        }

            //move the robot backwards until it is in front of a beacon
            while (odsSensor.getLightDetected() < .3 && (!super.isStopRequested())) {
                move(-mSpd);
                /*while (uLeft < uRight && (!super.isStopRequested())) {
                    leftTurn(trnSpd);
                    setUltra();
                    telemetry.addData("ultraL: ", uLeft);
                    telemetry.addData("ultraR: ", uRight);
                    telemetry.update();
                }*/
                telemetry.addData("ODS Light Detected: ", odsSensor.getLightDetected());
                telemetry.update();
            }

            stopMot();
            waiting(2000);

            pushBeacon(COLOR);

            stop();

        //move the robot forwards until it is in front of a beacon
        while (odsSensor.getLightDetected() < .3 && (!super.isStopRequested())) {
            move(mSpd);
            while (uLeft < uRight && (!super.isStopRequested())) {
                leftTurn(trnSpd);
                setUltra();
                telemetry.addData("ultraL: ", uLeft);
                telemetry.addData("ultraR: ", uRight);
                telemetry.update();
            }
            telemetry.addData("ODS Light Detected: ", odsSensor.getLightDetected());
            telemetry.update();
        }

        stopMot();
        waiting(2000);

        pushBeacon(COLOR);
    }

}
