package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp(name = "MecanumDrive",group="teleop")
public class MechanumWheelDriving extends OpMode {
    DcMotor leftFrontMotor;
    DcMotor rightFrontMotor;
    DcMotor leftBackMotor;
    DcMotor rightBackMotor;

    MecanumHardwareMap robot       = new MecanumHardwareMap();
    double fwdBackPower, strafePower, turnPower, maxPower;
    double leftFrontPower, rightFrontPower;
    double leftBackPower, rightBackPower;

    //leftFrontMotor = hardwareMap.dcMotor.get("leftFrontMotor");
    //rightFrontMotor = hardwareMap.dcMotor.get("rightFrontMotor");
    //leftBackMotor = hardwareMap.dcMotor.get("leftBackMotor");
    //rightBackMotor = hardwareMap.dcMotor.get("rightBackMotor");


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
        //telemetry.addData("powers",)
        robot.leftFrontMotor.setPower(leftFrontPower);
        robot.rightFrontMotor.setPower(rightFrontPower);
        robot.leftBackMotor.setPower(leftBackPower);
        robot.rightBackMotor.setPower(rightBackPower);

    }
    @Override
    public void stop() {
    }
}




