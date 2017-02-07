package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Radmin on 2/4/2017.
 */

/*This awesome program was created
by Randy with help from instructors*/
@TeleOp(name="MiniBot0", group="Iterative Opmode")  // @AutonomousSense(...) is the other common choice
//@Disabled
public class MiniBot0 extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();

    // Textrix motors

    public DcMotor motor1;
    public DcMotor motor2;
    public DcMotor motor3;
    public DcMotor motor4;

    //motor vectors
    double d1[] = {0.0, 1.0};
    double d2[] = {-1.0, 0};
    double d3[] = {0.0, -1.0};
    double d4[] = {1.0, 0.0};

    //scalar motor values
    double m1pwr;
    double m2pwr;
    double m3pwr;
    double m4pwr;

    @Override
    public void init() {

        telemetry.addData("Status", "Running: " + runtime.toString());

        motor1 = hardwareMap.dcMotor.get("motor1");
        motor2 = hardwareMap.dcMotor.get("motor2");
        motor3 = hardwareMap.dcMotor.get("motor3");
        motor4 = hardwareMap.dcMotor.get("motor4");
    }

    /*
    * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
    */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

        runtime.reset();

    }
    // comment
    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */

    @Override
    public void loop() {

        telemetry.addData("Status", "Running: " + runtime.toString());
        //left and right drive motors
        float joyStickY= -gamepad1.left_stick_y; //vertical axis power
        float joyStickX = gamepad1.left_stick_x; //horizontal axis power
        telemetry.addData("Joy stick Y",joyStickY);
        telemetry.addData("Joy Stick X",joyStickX);
        telemetry.update();

        //add some additional controls to the miniBot
        float rJStkX = gamepad1.right_stick_x; //rotational power

        //joy stick vector
        double jsk[] = {joyStickX, joyStickY};

        // motor powers (dot products)
        if(rJStkX >= 0.4 || rJStkX < -0.4){
            m1pwr = (rJStkX * d1[0]) + (rJStkX * d1[1]);
            m2pwr = (rJStkX * d2[0]) + (rJStkX * d2[1]);
            m3pwr = (rJStkX * d3[0]) + (rJStkX * d3[1]);
            m4pwr = (rJStkX * d4[0]) + (rJStkX * d4[1]);
        }
        else {
            m1pwr = (jsk[0] * d1[0]) + (jsk[1] * d1[1]);
            m2pwr = (jsk[0] * d2[0]) + (jsk[1] * d2[1]);
            m3pwr = (jsk[0] * d3[0]) + (jsk[1] * d3[1]);
            m4pwr = (jsk[0] * d4[0]) + (jsk[1] * d4[1]);
        }

        //set motor powers to the motors
        motor1.setPower(m1pwr);
        motor2.setPower(m2pwr);
        motor3.setPower(m3pwr);
        motor4.setPower(m4pwr);

    }
}
