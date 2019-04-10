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
public class BaseDrive extends LinearOpMode {
        //All hardware
        private DcMotor motorFwdLeft;
        private DcMotor motorFwdRight;
        private DcMotor motorBackLeft;
        private DcMotor motorBackRight;
        private DcMotor motorL;
        private DcMotor motorR;
        private DcMotor motorS;
        private DcMotor lift;
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
            motorFwdRight = hardwareMap.get(DcMotor.class, "motorFwdRight");
            motorBackLeft = hardwareMap.get(DcMotor.class, "motorBackLeft");
            motorFwdLeft = hardwareMap.get(DcMotor.class, "motorFwdLeft");
            motorBackRight = hardwareMap.get(DcMotor.class, "motorBackRight");

            motorL = hardwareMap.get(DcMotor.class, "motorL");
            motorR = hardwareMap.get(DcMotor.class, "motorR");
            motorS = hardwareMap.get(DcMotor.class, "motorS");
            lift = hardwareMap.get(DcMotor.class, "lift");

            mServo = hardwareMap.get(Servo.class, "mServo");
            distanceSensorL = hardwareMap.get(Rev2mDistanceSensor.class, "distanceSensorL");
            lights = hardwareMap.get(RevBlinkinLedDriver.class, "lights");
            telemetry.addData("Status", "Initialized");
            telemetry.update();

            //  Set encoder for arm and lift to zero.
            motorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //  Use the encoder reading to control the motor.
            motorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //  Send telemetry message to indicate successful Encoder reset
            telemetry.addData("Encoder", "Starting at %7d counts", motorL.getCurrentPosition());
            telemetry.update();


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

                motorFwdLeft.setPower(-left);
                motorFwdRight.setPower(right);
                motorBackLeft.setPower(left);
                motorBackRight.setPower(-right);

                /*These buttons use the encoders to move the mineral arm to a set location
                * A button must be held down in order for the arm to move
                * If a button is released, the arm stops moving
                * The joysticks and also be used to manually overide the encoders and fine tune positioning*/
                if(this.gamepad2.a){//retracted
                    motorL.setTargetPosition(getCountsPerDegree(0,1));
                    motorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motorL.setPower(1.0);

                    motorR.setTargetPosition(getCountsPerDegree(0,2));
                    motorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motorR.setPower(1.0);
                }
                else if(this.gamepad2.x){//up
                    motorL.setTargetPosition(getCountsPerDegree(-90,1));
                    motorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motorL.setPower(1.0);

                    motorR.setTargetPosition(getCountsPerDegree(65,2));
                    motorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motorR.setPower(1.0);
                }
                else if(this.gamepad2.y){//collecting
                    motorL.setTargetPosition(getCountsPerDegree(-187,1));
                    motorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motorL.setPower(1.0);

                    motorR.setTargetPosition(getCountsPerDegree(-44,2));
                    motorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motorR.setPower(1.0);
                }
                else if(this.gamepad2.b){//dropping
                    motorL.setTargetPosition(getCountsPerDegree(-77,1));
                    motorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motorL.setPower(1.0);

                    motorR.setTargetPosition(getCountsPerDegree(205,2));
                    motorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motorR.setPower(1.0);
                }
                /*These are the joystick controls
                * The left joystick controls the shoulder
                * The right joystick controls the elbow*/
                else if(this.gamepad2.left_stick_y != 0 || this.gamepad2.right_stick_y != 0){
                    motorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    motorL.setPower(this.gamepad2.left_stick_y);

                    motorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    motorR.setPower(this.gamepad2.right_stick_y);
                }
                //This stops the encoders if nothing is pressed
                else{
                    motorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    motorL.setPower(0);

                    motorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    motorR.setPower(0);
                }


                /*This section controls the intake wheel
                * The left trigger outtakes minerals
                * The right trigger intakes minerals
                * Only one can be pressed at a time or the motor stops moving*/
                if(this.gamepad2.right_trigger != 0 ^ this.gamepad2.left_trigger != 0){
                    if(this.gamepad2.left_trigger != 0){
                        motorS.setPower(this.gamepad2.left_trigger);
                    }
                    else if(this.gamepad2.right_trigger != 0){
                        motorS.setPower(-this.gamepad2.right_trigger);
                    }
                }
                else{
                    motorS.setPower(0);
                }

                /*This section controls the lift mechanism
                * The left trigger sets the lift to the retracted position
                * The left bumper lowers the lift without using encoders
                * The right trigger sets the lift to the extended position
                * The right bumper raises the lift without using encoders*/
                if(this.gamepad1.right_trigger != 0 ^ this.gamepad1.left_trigger != 0 ^ this.gamepad1.left_bumper != false ^ this.gamepad1.right_bumper != false){
                    if(this.gamepad1.right_trigger != 0){
                        lift.setTargetPosition(18250);
                        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        lift.setPower(1);
                    }
                    else if(this.gamepad1.left_trigger != 0){//down
                        lift.setTargetPosition(0);
                        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        lift.setPower(1);
                    }
                    else if(this.gamepad1.left_bumper != false){
                        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        lift.setPower(-1);
                    }
                    else if(this.gamepad1.right_bumper != false){
                        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        lift.setPower(1);
                    }

                }
                else{
                    lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    lift.setPower(0);
                }

                /*This section contols the marker basket to fix it if it gets in the way
                * The dpad moves it up and down*/
                if(this.gamepad1.dpad_up){
                    mServo.setPosition(mServo.getPosition()+0.01);
                }
                else if(this.gamepad1.dpad_down){
                    mServo.setPosition(mServo.getPosition()-0.01);
                }

                //Telemetry shows all of the angles and positions of encoders and servos
                telemetry.addData("LiftPos", lift.getCurrentPosition());
                telemetry.addData("motorR", motorR.getCurrentPosition()/COUNTS_PER_DEGREE2);
                telemetry.addData("motorL", motorL.getCurrentPosition()/COUNTS_PER_DEGREE1);
                telemetry.addData("Intake Power" , this.gamepad2.right_trigger);
                telemetry.addData("mServo Pos", mServo.getPosition());
                telemetry.addData("distance", distanceSensorL.getDistance(DistanceUnit.CM));
                telemetry.addData("Status", "Running");
                telemetry.update();

            }
        }
}
