package common;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


public class Limelight {

    public enum Pipeline  {LOCATION, APRIL_TAG }
    private final Limelight3A limelight;


    public Limelight (LinearOpMode opMode) {
        limelight = opMode.hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();

        LLStatus status = limelight.getStatus();
        Logger.message("%s started", status.getName());
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
        for (int i = 0; i < 3; i++) {
            LLResult result = limelight.getLatestResult();
            if (result.isValid()) {
                double tx = result.getTx();
                Logger.message("Limelight Tx: %5.2f", tx);
                return tx;
            }
            Logger.warning("Limelight Tx is invalid");
        }
        return 0;
    }

    public double GetTargetArea() {
        LLResult result = limelight.getLatestResult();
        if (result.isValid()) {
            double ta = result.getTa();
            //Logger.message("Limelight target Area: %5.2f", ta);
            return ta;
        }
        Logger.warning("Limelight Tx is invalid");
        return 0;
    }

    public void setPipeline (Pipeline pipeline) {

        int index = -1;
        String type = "";

        switch (pipeline) {
            case APRIL_TAG:
                index = 5;
                type = "pipe_fiducial";
                break;

            case LOCATION:
                index = 3;
                type = "pipe_location";
                break;
        }

        limelight.pipelineSwitch(index);
        LLStatus status = limelight.getStatus();
        Logger.message("pipeline index: %d, type: %s", status.getPipelineIndex(), status.getPipelineType());
        if (status.getPipelineIndex() != index || !status.getPipelineType().equals(type)) {
            Logger.warning("%s pipeline not set to type %s", pipeline, type);
        }
    }
}
