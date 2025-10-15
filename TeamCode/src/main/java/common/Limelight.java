package common;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


public class Limelight {

    public enum Pipeline  {LOCATION, APRIL_TAG, }
    private final Limelight3A limelight;

    private int pipeline = 5;

    public Limelight (LinearOpMode opMode) {
        limelight = opMode.hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(pipeline);
        limelight.start();
    }

    /**
     * Get the field relative pose of the robot using Apriltags.
     *
     * @param heading heading -180 to 180
     * @return pose - origin is center to the field
     */
    public Pose2D getPosition (double heading) {

        Pose2D pose = null;
        limelight.updateRobotOrientation(heading);
        LLResult result = limelight.getLatestResult();
        if (result.isValid()) {
            if (result.isValid()) {
                Pose3D robotPose = result.getBotpose_MT2();
                Position position = robotPose.getPosition();
                YawPitchRollAngles orientation = robotPose.getOrientation();
                assert (position.unit == DistanceUnit.METER);
                double METERS_TO_INCHES = 39.3701;
                double x = position.x * METERS_TO_INCHES;
                double y = position.y * METERS_TO_INCHES;

                pose = new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, orientation.getYaw(AngleUnit.DEGREES));
            }
        }
        return pose;
    }

    public double GetTx () {
        LLResult result = limelight.getLatestResult();
        if (result.isValid()) {
            return result.getTx();
        }
        return 0;
    }

    public void setPipeline (Pipeline pipeline) {

        switch (pipeline) {
            case APRIL_TAG:
                this.pipeline = 5;
                break;

            case LOCATION:
                this.pipeline = 3;
                break;
        }

        limelight.pipelineSwitch(this.pipeline);
    }
}
