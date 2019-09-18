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

    double now, before;

    double LeftFrontdPrev, LeftFrontdCurrent;

    double RightFrontdPrev, RightFrontdCurrent;

    double LeftBackdPrev, LeftBackdCurrent;

    double RightBackdPrev, RightBackdCurrent;


    @Override
    public void init() {

        robot.init(hardwareMap);
    }
    @Override
    public void init_loop(){
        now = 0.0;
        before=0.0;

        LeftFrontdPrev=0.0;
        LeftFrontdCurrent=0.0;

        RightFrontdPrev=0.0;
        RightFrontdCurrent=0.0;

        LeftBackdPrev=0.0;
        LeftBackdCurrent=0.0;

        RightBackdPrev=0.0;
        RightBackdCurrent=0.0;
    }


    @Override
    public void start() {

    }
    @Override
    public void loop() {
        //Sense
            fwdBackPower = -gamepad1.left_stick_y;
            strafePower = gamepad1.left_stick_x;
            turnPower = gamepad1.right_stick_x;


            before=now;
            now=elapsed_time.milliseconds();


            //leftFront
            LeftFrontdPrev=LeftFrontdCurrent;
            LeftFrontdCurrent=robot.leftFrontMotor.getCurrentPosition();


            //leftBack
            LeftBackdPrev = LeftBackdCurrent;
            LeftBackdCurrent=robot.leftBackMotor.getCurrentPosition();


            //rightFront
            RightFrontdPrev=RightBackdCurrent;
            RightFrontdCurrent=robot.rightFrontMotor.getCurrentPosition();


            //rightBack
            RightBackdPrev=RightBackdCurrent;
            RightBackdCurrent=robot.rightBackMotor.getCurrentPosition();


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






        //leftFront
        telemetry.addData("leftFront Power :", leftFrontPower);
        telemetry.addData("leftFront Time :", now);
        telemetry.addData("leftFront Encoders :", LeftFrontdCurrent);
        telemetry.addData("leftFront velocity :", (LeftFrontdCurrent-LeftFrontdPrev)/(now-before));
        telemetry.addData(" "," ");

        //leftBack
        telemetry.addData("leftBack Power :", leftBackPower);
        telemetry.addData("leftBack Time :", now);
        telemetry.addData("leftBack Encoders :",LeftBackdCurrent);
        telemetry.addData("leftBack velocity :",(LeftBackdCurrent-LeftBackdPrev)/(now-before));

        //rightFront
        telemetry.addData("rightFront Power :", rightFrontPower);
        telemetry.addData("rightFront Time :", now);
        telemetry.addData("rightFront Encoders :", RightFrontdCurrent);
        telemetry.addData("rightFront velocity :",(RightFrontdCurrent-RightFrontdPrev)/(now-before));
        telemetry.addData(" "," ");

        //rightBack
        telemetry.addData("rightBack Power :",rightBackPower);
        telemetry.addData("rightBack Time :", now);
        telemetry.addData("rightBack Encoders :",RightBackdCurrent);
        telemetry.addData("rightBack velocity :",(RightBackdCurrent-RightBackdPrev)/(now-before));
        telemetry.addData(" "," ");

        //moving
        robot.leftFrontMotor.setPower(leftFrontPower);
        robot.rightFrontMotor.setPower(rightFrontPower);
        robot.leftBackMotor.setPower(leftBackPower);
        robot.rightBackMotor.setPower(rightBackPower);

    }
    @Override
    public void stop() {
    }
}


