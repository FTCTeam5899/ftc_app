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
public class MotorTest extends LinearOpMode {

    private DcMotor motorL;
    private DcMotor motorR;
    private DcMotor motorS;

    @Override
    public void runOpMode() {

        motorL = hardwareMap.get(DcMotor.class, "motorL");
        motorR = hardwareMap.get(DcMotor.class, "motorR");
        motorS = hardwareMap.get(DcMotor.class, "motorS");

        double max = 1;
        double pow = 0;
        double pow2 = 0;
        double pow3 = 0;

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            pow = -this.gamepad1.left_stick_y;
            pow2 = -this.gamepad1.right_stick_y;

            if(this.gamepad1.dpad_up){pow3 = 1;}
            else if(this.gamepad1.dpad_down){pow3 = -1;}
            else{pow3 = 0;}


            if(this.gamepad1.x){max = 0.3;}
            if(this.gamepad1.y){max = 0.5;}
            if(this.gamepad1.b){max = 0.8;}
            if(this.gamepad1.a){max = 1.0;}

            motorL.setPower(pow * max);
            motorR.setPower(pow2 * max);
            motorS.setPower(pow3);

            telemetry.addData("Max Speed", max);
            telemetry.addData("Status", "Running");
            telemetry.update();

        }
    }
}
