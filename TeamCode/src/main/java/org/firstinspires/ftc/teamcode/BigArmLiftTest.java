package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp
@Disabled
public class BigArmLiftTest extends LinearOpMode {

    private DcMotor motorFwdLeft;
    private DcMotor motorFwdRight;

    private static double left;
    private static double right;
    private double max = 1.0;

    @Override

    public void runOpMode() {

        motorFwdRight = hardwareMap.get(DcMotor.class, "motorFwdRight");

        motorFwdLeft = hardwareMap.get(DcMotor.class, "motorFwdLeft");


        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if(this.gamepad1.x && this.gamepad1.y && this.gamepad1.b && this.gamepad1.a && max == 1.0){max = 0.3;}
            else if(this.gamepad1.x && this.gamepad1.y && this.gamepad1.b && this.gamepad1.a){max = 1;}

            left = -this.gamepad1.left_stick_y * max;
            right = this.gamepad1.right_stick_y * max;

            motorFwdLeft.setPower(left);
            motorFwdRight.setPower(right);

            //telemetry.addData("Nathan Mode", mode);

            telemetry.addData("Status", "Running");
            telemetry.update();

        }
    }
}

