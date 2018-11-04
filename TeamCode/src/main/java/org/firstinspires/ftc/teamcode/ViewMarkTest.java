package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

@TeleOp
public class ViewMarkTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        params.vuforiaLicenseKey = "Ac8qVVb/////AAABmW0ZY5qaKUVegMYq2LOSDO1OzcyAP6IoQTVXJ5E6V+Xier9" +
                "dD5quzzS0toHeXCyiWZn6Wsw2WdgS9GLwIjNfmuozNwBTuU9DBkABBpyBwAXiiZmzTgLLkNR1dw9+Vwl/S7" +
                "6TuqcaNHTl8vvQOTssFkIvXC0f5acepwlTL8xjEsvb3Y6Fys/mMQprOuhg/9f44K5DsQwutOaTrsVjGyJ1f" +
                "WyT6cDM+BPqLcBs+/oisbHud/8Q8Iz3I/9+xXJW1ZChn659VoZ0a2Sdoa5FdLl72OpVEzA+d+lYaGcZXmE8" +
                "NszlxxdOivvNkcFfF45zRyqisSfGowjpyFglNBSWTsNiD1shkpP0uyoeK9lRVxIE4Qug";
        params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;

        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(params);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);

        VuforiaTrackables beacons = vuforia.loadTrackablesFromAsset("RoverRuckus");
        beacons.get(0).setName("Rover");
        beacons.get(1).setName("Foot");
        beacons.get(2).setName("Mars");
        beacons.get(3).setName("Space");

        waitForStart();

        beacons.activate();

        while(opModeIsActive()){
            for(VuforiaTrackable beac : beacons){
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) beac.getListener()).getPose();

                if(pose != null){
                    VectorF translation = pose.getTranslation();

                    telemetry.addData(beac.getName() + "-Translation", translation);
                    double degreesToTurn = Math.toDegrees(Math.atan2(translation.get(1), translation.get(2)));

                    telemetry.addData(beac.getName() + "-Degrees", degreesToTurn);

                }
            }
            telemetry.update();
        }
    }
}
