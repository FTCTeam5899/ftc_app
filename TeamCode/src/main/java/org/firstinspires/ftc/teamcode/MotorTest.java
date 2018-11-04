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
public class MotorTest extends LinearOpMode {

    private DcMotor motor;

    @Override
    public void runOpMode() {

        motor = hardwareMap.get(DcMotor.class, "motor");
        double max = 1;
        double left = 0;

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            left = -this.gamepad1.left_stick_y;

            if(this.gamepad1.x){max = 0.3;}
            if(this.gamepad1.y){max = 0.5;}
            if(this.gamepad1.b){max = 0.8;}
            if(this.gamepad1.a){max = 1.0;}

            motor.setPower(left * max);

            telemetry.addData("Max Speed", max);
            telemetry.addData("Status", "Running");
            telemetry.update();

        }
    }
}
