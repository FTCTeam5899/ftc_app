package org.firstinspires.ftc.teamcode;
import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="stop_test", group="Official")
public class stop_test extends AutoSupplies{
    @Override
    public void runOpMode() {
        enableGoldDetector();
        telemetry.addData("Status", "DogeCV 2018.0 - Gold Aligner");

        initForAutonomous();
        double x = 0;
        double times = 0;
        double tTime = 0;
        double angle = getAngle();
        double left = 0;
        double right = 0;
        //  Wait until start
        waitForStart();
        resetPitch();
        telemetry.clear();
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE_VIOLET);
        //locks servo in place
        mServo.setPosition(0.33);
        while(!isStopRequested()){
            telemetry.addData("pitch", getPitch());
            telemetry.addData("distance", distanceSensorL.getDistance(DistanceUnit.CM));
            telemetry.update();

            left = this.gamepad1.left_stick_y;
            right = this.gamepad1.right_stick_y;

            motorFwdLeft.setPower(-left);
            motorFwdRight.setPower(right);
            motorBackLeft.setPower(left);
            motorBackRight.setPower(-right);
        }
        motorBackLeft.setPower(-0);
        motorFwdLeft.setPower(0);
        motorBackRight.setPower(0);
        motorFwdRight.setPower(-0);
    }
}

