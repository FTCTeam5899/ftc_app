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

@TeleOp
public class BaseDrive extends LinearOpMode {

        private DcMotor motorFwdLeft;
        private DcMotor motorFwdRight;
        private DcMotor motorBackLeft;
        private DcMotor motorBackRight;
        private DcMotor motorL;
        private DcMotor motorR;
        private DcMotor motorS;

        private Servo mServo;
        private RevBlinkinLedDriver lights;

        private static double left;
        private static double right;

        private double max = 1.0;

        @Override
        public void runOpMode() {

            motorFwdRight = hardwareMap.get(DcMotor.class, "motorFwdRight");
            motorBackLeft = hardwareMap.get(DcMotor.class, "motorBackLeft");
            motorFwdLeft = hardwareMap.get(DcMotor.class, "motorFwdLeft");
            motorBackRight = hardwareMap.get(DcMotor.class, "motorBackRight");

            motorL = hardwareMap.get(DcMotor.class, "motorL");
            motorR = hardwareMap.get(DcMotor.class, "motorR");
            motorS = hardwareMap.get(DcMotor.class, "motorS");

            mServo = hardwareMap.get(Servo.class, "mServo");
            lights = hardwareMap.get(RevBlinkinLedDriver.class, "lights");
            telemetry.addData("Status", "Initialized");
            telemetry.update();
            // Wait for the game to start (driver presses PLAY)
            waitForStart();

            // run until the end of the match (driver presses STOP)
            while (opModeIsActive()) {
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_RAINBOW_PALETTE);
                if(this.gamepad1.x && this.gamepad1.y && this.gamepad1.b && this.gamepad1.a && max == 1.0){max = 0.3;}
                else if(this.gamepad1.x && this.gamepad1.y && this.gamepad1.b && this.gamepad1.a){max = 1;}


                left = this.gamepad1.left_stick_y * max;
                right = this.gamepad1.right_stick_y * max;


                motorFwdLeft.setPower(-left);
                motorFwdRight.setPower(right);
                motorBackLeft.setPower(left);
                motorBackRight.setPower(-right);

                motorL.setPower(this.gamepad2.left_stick_y);
                motorR.setPower(this.gamepad2.right_stick_y);
                if(this.gamepad2.right_trigger != 0 ^ this.gamepad2.left_trigger != 0){
                    if(this.gamepad2.right_trigger != 0){
                        motorS.setPower(this.gamepad2.right_trigger);
                    }
                    else if(this.gamepad2.left_trigger != 0){
                        motorS.setPower(-this.gamepad2.left_trigger);
                    }
                }
                else{
                    motorS.setPower(0);
                }



                if(this.gamepad1.dpad_up){
                    mServo.setPosition(mServo.getPosition()+0.01);
                }
                else if(this.gamepad1.dpad_down){
                    mServo.setPosition(mServo.getPosition()-0.01);
                }

                telemetry.addData("Intake Power" , this.gamepad2.right_trigger);
                telemetry.addData("mServo Pos", mServo.getPosition());
                telemetry.addData("Status", "Running");
                telemetry.update();

            }
        }
}
