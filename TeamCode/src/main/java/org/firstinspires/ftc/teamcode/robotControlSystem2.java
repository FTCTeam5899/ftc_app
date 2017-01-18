package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Radmin on 1/5/2017.
 */

        import com.qualcomm.robotcore.eventloop.opmode.OpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.eventloop.opmode.Disabled;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.util.ElapsedTime;
        import com.qualcomm.robotcore.hardware.DcMotorController;
        import com.qualcomm.robotcore.hardware.DcMotorController;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 *
 * Updated 12/2/2016
 */

@TeleOp(name="RobotControlSystem2", group="Iterative Opmode")  // @Autonomous(...) is the other common choice
//@Disabled
public class robotControlSystem2 extends OpMode
{
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftMotor = null;
    private DcMotor leftMotor2 = null;
    private DcMotor rightMotor = null;
    private DcMotor rightMotor2 = null;
    private DcMotor capBallLift = null;
    private Servo rServo = null;

    private float motorSlowdownFactor = 1;

    final static int ENCODER_CPR = 1120;    //Encoder counts per Revolution
    int degrees = 180; //sets the degrees we want the motor to turn
    double counts = (double) degrees * ENCODER_CPR/360.0; //sets the amount of counts for the motor to turn to tun the spesified derees

    //functions for The Dominator (Robot 2)
    public void leftMotors(float pow){
        leftMotor.setPower(-pow);
        leftMotor2.setPower(leftMotor.getPower());
    }

    public void  rightMotors(float pow){
        rightMotor.setPower(-pow);
        rightMotor2.setPower(rightMotor.getPower());
    }

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() { /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
        leftMotor  = hardwareMap.dcMotor.get("left_drive");
        leftMotor2 = hardwareMap.dcMotor.get("left_drive2");
        rightMotor = hardwareMap.dcMotor.get("right_drive");
        rightMotor2 = hardwareMap.dcMotor.get("right_drive2");
        capBallLift = hardwareMap.dcMotor.get("capLift");
        rServo = hardwareMap.servo.get("forkDrop");

        // set the drop servo's postition
        double position = 1;
        rServo.setPosition(position);

        // eg: Set the drive motor directions:
        // Reverse the motor that runs backwards when connected directly to the battery
        // leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        rightMotor2.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        //catapultMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

        runtime.reset();

    }
    // comment
    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    boolean aButtonOn = false;
    @Override
    public void loop() {

        telemetry.addData("Status", "Running: " + runtime.toString());
        //left and right drive motors
        float leftY = -gamepad1.left_stick_y; //power for left_motor attached to left controller stick
        float rightY = -gamepad1.right_stick_y; //poewr for right_motor attached to right controller stick
        telemetry.addData("Left Gamepad",leftY);
        telemetry.addData("Right Gamepad",rightY);
        //capBallLift
        float rightY2 = -gamepad2.right_stick_y;

        //button to slow down the drive motors on the robot
        //button must be held to be in slowdown mode
        //when the left bumper is released, the motor power will return to normal
        boolean leftBumper1 = gamepad1.left_bumper;
        if (leftBumper1) {
            motorSlowdownFactor = (float) 0.5;
        }
        else {
            motorSlowdownFactor = (float) 1.0;
        }

        //button to switch the driving direction of the robot
        boolean buttonA = gamepad1.a;
        if(buttonA){
            aButtonOn = !aButtonOn;
        }
        //Button for moving the CapBallLift Drop
        boolean buttonA2 = gamepad2.a;
        boolean buttonB2 = gamepad2.b;
        boolean buttonX2 = gamepad2.x;

        // eg: Run wheels in tank mode (note: The joystick goes negative when pushed forwards)
        //leftMotor.setPower(leftY); got rid of original code to replace it with code that
        //rightMotor.setPower(rightY); works with the Dominater(robot 2)
        if(!aButtonOn){
            //reverse the direction the robot moves
            leftMotors(-rightY *(float).7 * motorSlowdownFactor);
            rightMotors(-leftY *(float).7 * motorSlowdownFactor);
        }
        else {
            leftMotors(leftY * motorSlowdownFactor);
            rightMotors(rightY * motorSlowdownFactor);
        }

        //run the capBallLift
        if ((rightY2>0.1) || (rightY2<-0.1))
        {
            // Make sure that servo is not in the way of the cap ball lifter
            rServo.setPosition(.5);
        }
        capBallLift.setPower(rightY2);

        //run the Drop servo
        //telemetry.addData("Position: ", position);
        //telemetry.update();
        if(buttonA2){
            rServo.setPosition(.3);
        }
        if(buttonB2) {
            rServo.setPosition(.5);
        }
        if(buttonX2){
            rServo.setPosition(1);
        }
    }

    /*
        * Code to run ONCE after the driver hits STOP
        */
    @Override
    public void stop() {
    }

}
