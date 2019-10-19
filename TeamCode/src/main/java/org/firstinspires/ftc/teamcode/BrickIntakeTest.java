package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@TeleOp
public class BrickIntakeTest extends LinearOpMode {
    //All hardware
    private DcMotor motorLeftIntake;
    private DcMotor motorRightIntake;
    private DcMotor motorSlideLeft;

    protected Rev2mDistanceSensor distanceSensorL;

    private Servo mServo;
    private RevBlinkinLedDriver lights;

    private static double left;
    private static double right;

    private double max = 1.0;
    //Encoder Specs
    //We keep these motors separate in case a change is made to motors or gear boxes

    //  Neverest 40 motor spec:  quadrature encoder, 280 pulses per revolution, count = 280 *4
    private static final double COUNTS_PER_MOTOR_REV = 1120;    // Neverest 40 motor encoder                    Left(shoulder) motor
    private static final double DRIVE_GEAR_REDUCTION1 = 13.5;     // This is < 1.0 if geared UP
    private static final double COUNTS_PER_DEGREE1 = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION1) / 360;

    //  Neverest 40 motor right spec:  quadrature encoder, 280 pulses per revolution, count = 280 *4
    private static final double COUNTS_PER_MOTOR_REV2 = 1120;    // Neverest 40 motor encoder                   Right(elbow) motor
    private static final double DRIVE_GEAR_REDUCTION2 = 13.5;     // This is < 1.0 if geared UP
    private static final double COUNTS_PER_DEGREE2 = (COUNTS_PER_MOTOR_REV2 * DRIVE_GEAR_REDUCTION2) / 360;

    //  Neverest 60 motor left spec:  quadrature encoder, 420 pulses per revolution, count = 420 *4             We do not currently use 60's but have in the past
    //private static final double COUNTS_PER_MOTOR_REV = 1680;    // Neverest 60 motor encoder                  This is just a reserve in case changes are made
    //private static final double DRIVE_GEAR_REDUCTION1 = 27.0;     // This is < 1.0 if geared UP
    //private static final double COUNTS_PER_DEGREE1 = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION1) / 360;

    /*This function determines the number of ticks a motor
     would need to move in order to achieve a certain degree*/
    private int getCountsPerDegree(double degrees, int motorNumber){
        int ans = 0;
        if(motorNumber == 1){
            ans = (int)(degrees * COUNTS_PER_DEGREE1);
        }
        else if(motorNumber == 2){
            ans = (int)(degrees * COUNTS_PER_DEGREE2);
        }
        else{
            return 1;
        }
        return ans;
    }
    @Override
    public void runOpMode() {
        //Prepares all the hardware
        motorRightIntake = hardwareMap.get(DcMotor.class, "motorRightintake");
        motorSlideLeft = hardwareMap.get(DcMotor.class, "motorBackLeft");
        motorLeftIntake = hardwareMap.get(DcMotor.class, "motorLeftIntake");


        mServo = hardwareMap.get(Servo.class, "mServo");
        distanceSensorL = hardwareMap.get(Rev2mDistanceSensor.class, "distanceSensorL");
        lights = hardwareMap.get(RevBlinkinLedDriver.class, "lights");
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //  Set encoder for arm and lift to zero.
        //  Send telemetry message to indicate successful Encoder reset
        // Wait for the game to start (driver presses PLAY)
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_RAINBOW_PALETTE);

            /*toggles the slow/demo mode
             * All 4 dpad buttons must be pressed at the same time to activate this*/
            if(this.gamepad1.x && this.gamepad1.y && this.gamepad1.b && this.gamepad1.a && max == 1.0){max = 0.3;}
            else if(this.gamepad1.x && this.gamepad1.y && this.gamepad1.b && this.gamepad1.a){max = 1;}

            /*Controls the speed for the 4 base drive motors
             * The left stick controls the left chain and the right the right*/
            left = this.gamepad1.left_stick_y * max;
            right = this.gamepad1.right_stick_y * max;

            motorLeftIntake.setPower(-left);
            motorRightIntake.setPower(right);
            motorSlideLeft.setPower(left);

            /*These buttons use the encoders to move the mineral arm to a set location
             * A button must be held down in order for the arm to move
             * If a button is released, the arm stops moving
             * The joysticks and also be used to manually overide the encoders and fine tune positioning*/




            /*These are the joystick controls
             * The left joystick controls the shoulder
             * The right joystick controls the elbow*/



            /*This section controls the intake wheel
             * The left trigger outtakes minerals
             * The right trigger intakes minerals
             * Only one can be pressed at a time or the motor stops moving*/


            /*This section controls the lift mechanism
             * The left trigger sets the lift to the retracted position
             * The left bumper lowers the lift without using encoders
             * The right trigger sets the lift to the extended position
             * The right bumper raises the lift without using encoders*/


            /*This section contols the marker basket to fix it if it gets in the way
             * The dpad moves it up and down*/
            if(this.gamepad1.dpad_up){
                mServo.setPosition(mServo.getPosition()+0.01);
            }
            else if(this.gamepad1.dpad_down){
                mServo.setPosition(mServo.getPosition()-0.01);
            }

            //Telemetry shows all of the angles and positions of encoders and servos
            telemetry.addData("Intake Power" , this.gamepad2.right_trigger);
            telemetry.addData("mServo Pos", mServo.getPosition());
            telemetry.addData("distance", distanceSensorL.getDistance(DistanceUnit.CM));
            telemetry.addData("Status", "Running");
            telemetry.update();

        }
    }
}
