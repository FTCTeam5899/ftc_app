package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@TeleOp(name = "MecanumDrive",group="teleop")

public class MecanumWheelDriving extends OpMode {

    MecanumHardwareMap robot       = new MecanumHardwareMap();
    double fwdBackPower, strafePower, turnPower, maxPower;
    double leftFrontPower, rightFrontPower;
    double leftBackPower, rightBackPower;

    private ElapsedTime elapsed_time = new ElapsedTime();



    @Override
    public void init() {

        robot.init(hardwareMap);
    }
    @Override
    public void init_loop(){
    }


    @Override
    public void start() {

    }
    @Override
    public void loop() {
        fwdBackPower = -gamepad1.left_stick_y;
        strafePower = gamepad1.left_stick_x;
        turnPower = gamepad1.right_stick_x;

        leftFrontPower = fwdBackPower + turnPower +strafePower;
        rightFrontPower = fwdBackPower - turnPower - strafePower;
        leftBackPower = fwdBackPower + turnPower - strafePower;
        rightBackPower = fwdBackPower - turnPower + strafePower;

        maxPower = Math.abs(leftFrontPower);
        if(Math.abs(rightFrontPower)>maxPower) {
            maxPower = Math.abs(rightFrontPower);
        }
        if(Math.abs(leftBackPower)>maxPower) {
            maxPower = Math.abs(leftBackPower);
        }
        if(Math.abs(rightBackPower)>maxPower) {
            maxPower = Math.abs(rightBackPower);
        }
        if(maxPower>1) {
            leftFrontPower = leftFrontPower/maxPower;
            rightFrontPower = rightFrontPower/maxPower;
            leftBackPower = leftBackPower/maxPower;
            rightBackPower = rightBackPower/maxPower;

        }
        telemetry.addData("leftFront:", leftFrontPower);
        telemetry.addData("leftFront", elapsed_time.milliseconds());
        //telemetry.addData("leftFront",
        telemetry.addData("leftBack:", leftBackPower);
        telemetry.addData("leftBack", elapsed_time.milliseconds());
        //telemetry.addData("leftBack",
        telemetry.addData("rightFront:", rightFrontPower);
        telemetry.addData("rightFront", elapsed_time.milliseconds());
        //telemetry.addData("rightFront",
        telemetry.addData("rightBack:",rightBackPower);
        telemetry.addData("rightBack", elapsed_time.milliseconds());
        //telemetry.addData("rightBack",
        robot.leftFrontMotor.setPower(leftFrontPower);
        robot.rightFrontMotor.setPower(rightFrontPower);
        robot.leftBackMotor.setPower(leftBackPower);
        robot.rightBackMotor.setPower(rightBackPower);

    }
    @Override
    public void stop() {
    }
}


