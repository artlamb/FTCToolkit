package common;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;

import utils.Pose;


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
     * Get the field relative pose of the robot using Apriltags and MagTag2.
     *
     * @param heading heading -180 to 180
     * @return pose - origin is center to the field
     */
    public Pose getPosition (double heading) {

        Pose pose = null;

        // convert to FTC field coordinates
        double ftcHeading = AngleUnit.normalizeRadians(heading - Math.toRadians(90));
        limelight.updateRobotOrientation(ftcHeading);

        LLResult result = limelight.getLatestResult();
        if (result.isValid()) {
            Pose3D robotPose = result.getBotpose_MT2();
            if (robotPose != null) {
                Position position = robotPose.getPosition();

                // convert from FTC field coordinates
                double x = position.unit.toInches(position.y);
                double y = -position.unit.toInches(position.x);

                YawPitchRollAngles orientation = robotPose.getOrientation();
                double h = orientation.getYaw(AngleUnit.RADIANS);

                pose = new Pose(x, y, h);
            }
        }
        return pose;
    }


    /**
     * Get the field relative pose of the robot using Apriltags.
     *
     * @return pose - origin is center to the field
     */
    public Pose getPosition () {

        Pose pose = null;

        LLResult result = limelight.getLatestResult();
        if (result.isValid()) {
            Pose3D robotPose = result.getBotpose();
            if (robotPose != null) {
                Position position = robotPose.getPosition();

                // convert from FTC field coordinates
                double x = position.unit.toInches(position.y);
                double y = -position.unit.toInches(position.x);

                YawPitchRollAngles orientation = robotPose.getOrientation();
                double h = orientation.getYaw(AngleUnit.RADIANS);
                h = AngleUnit.normalizeRadians(h - Math.toRadians(90));
                pose = new Pose(x, y, h);
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
            return result.getTa();
        }
        Logger.warning("Limelight Tx is invalid");
        return 0;
    }

    public int GetAprilTagID() {
        LLResult result = limelight.getLatestResult();
        if (result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            if (fiducialResults.size() == 1) {
                return fiducialResults.get(0).getFiducialId();
            }
        }

        //Logger.warning("No April Tag found");
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
                type = "pipe_fiducial";
                break;
        }

        long startTime = System.currentTimeMillis();
        long timeout = 1000;
        while (true) {
            limelight.pipelineSwitch(index);
            LLStatus status = limelight.getStatus();

            if (status.getPipelineIndex() == index && status.getPipelineType().equals(type)) {
                Logger.message("%s pipeline set to index: %d, type: %s in %d ms",
                        pipeline, index, type, System.currentTimeMillis() - startTime);
                break;
            }

            if (System.currentTimeMillis() - startTime > timeout) {
                Logger.warning("pipeline index: %d, type: %s", status.getPipelineIndex(), status.getPipelineType());
                Logger.warning("%s pipeline not set to index: %d, type %s", pipeline, index, type);
                Logger.warning("set pipeline timed out after %d ms", timeout);
                break;
            }
        }
    }
}
