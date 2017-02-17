package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Radmin on 2/6/2017.
 */

@TeleOp(name="SensorTest", group="Iterative Opmode")  // @AutonomousSense(...) is the other common choice
//@Disabled
public class SensorTest extends OpMode {


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
    ColorSensor colorSen1;
    ColorSensor colorSen2;

    //other variables

    float mSpd = 1f;

    int[] cValues1 = new int[3];
    double cValues2[] = new double[3];


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
    public  void init(){

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
        colorSen1 = hardwareMap.colorSensor.get("colorSen1");
        colorSen2 = hardwareMap.colorSensor.get("colorSen2");


        // eg: Set the drive motor directions:
        // Reverse the motor that runs backwards when connected directly to the battery
        // leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        rightMotor2.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        odsSensor.enableLed(true);

    }

    @Override
    public void start() {

        runtime.reset();

    }

    @Override
    public void loop() {

        telemetry.addData("Status", "Running: " + runtime.toString());
        //left and right drive motors
        float leftY = -gamepad1.left_stick_y; //power for left_motor attached to left controller stick
        float rightY = -gamepad1.right_stick_y; //poewr for right_motor attached to right controller stick
        telemetry.addData("Left Gamepad", leftY);
        telemetry.addData("Right Gamepad", rightY);

        leftMotors(leftY);
        rightMotors(rightY);

        double odsLight = odsSensor.getLightDetected();

        telemetry.addData("Ods Light: ", odsLight);
        //telemetry.update();

        double dist1 = ultra1.getUltrasonicLevel();
        double dist2 = ultra2.getUltrasonicLevel();

        telemetry.addData("Ultra1: ", dist1);
        telemetry.addData("Ultra2: ", dist2);
        //telemetry.update();

        cValues1[0] =  colorSen1.red();
        cValues1[1] =  colorSen1.green();
        cValues1[2] =  colorSen1.blue();

        telemetry.addData("RGB Color1: ", cValues1.toString());

        cValues2[0] = colorSen2.red();
        cValues2[1] = colorSen2.green();
        cValues2[2] = colorSen2.blue();

        telemetry.addData("RGB Color2: ", cValues2.toString());

        telemetry.update();




    }

}
