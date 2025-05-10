package common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import drivers.GoBildaPinpointDriver;
import utils.Pose;


public class Pinpoint {

    private final GoBildaPinpointDriver pinpointDrive;
    private Pose startPose;

    private static final double GOBILDA_4_BAR_POD    = 19.89436789f; //ticks-per-mm for the goBILDA 4-Bar Pod
    public  static double ENCODER_SCALE = 1.0;

    public Pinpoint(LinearOpMode opMode) {

        pinpointDrive = opMode.hardwareMap.get(GoBildaPinpointDriver.class,Config.PINPOINT);

        // Set encoder directions
        pinpointDrive.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);

        // Set tracking point to the center of the robot (in mm)
        pinpointDrive.setOffsets(-84, 0, DistanceUnit.MM);

        // Scale the encoder resolution if necessary
        if (ENCODER_SCALE == 1.0)
            pinpointDrive.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        else
            pinpointDrive.setEncoderResolution(GOBILDA_4_BAR_POD * ENCODER_SCALE, DistanceUnit.MM);

        //TODO: If you find that the gobilda Yaw Scaling is incorrect you can edit this here
        //  pinpoint.setYawScalar(1.0);

        setStartPose(new Pose());
    }

    /**
     * This sets the start pose. Changing the start pose should move the robot as if all its
     * previous movements were displacing it from its new start pose.
     *
     * @param setStart the new start pose
     */
    public void setStartPose(Pose setStart) {
        startPose = setStart;
    }

    /**
     * This sets the current pose estimate. Changing this should just change the robot's current
     * pose estimate, not anything to do with the start pose.
     *
     * @param setPose the new current pose estimate
     */
    public void setPose(Pose setPose) {
        Pose setPinpointPose = Pose.subtract(setPose, startPose);
        pinpointDrive.setPosition(new Pose2D(DistanceUnit.INCH, setPinpointPose.getX(), setPinpointPose.getY(), AngleUnit.RADIANS, setPinpointPose.getHeading()));
    }

    /**
     * This returns the current pose estimate.
     *
     * @return returns the current pose estimate as a Pose
     */
    public Pose getPose() {
        Pose2D rawPose = pinpointDrive.getPosition();
        Pose pose = new Pose(rawPose.getX(DistanceUnit.INCH), rawPose.getY(DistanceUnit.INCH), rawPose.getHeading(AngleUnit.RADIANS));
        return Pose.add(startPose, Pose.rotatePose(pose, startPose.getHeading(), false));
    }

    public void update() {
        pinpointDrive.update();
    }

    /**
     * This resets the IMU.
     */
    public void resetIMU() {
        pinpointDrive.recalibrateIMU();
        waitUnitReady();
    }

    /**
     * This resets the imu and position.
     */
    public void resetPinpoint(){
        pinpointDrive.resetPosAndIMU();
        waitUnitReady();
    }

    public void waitUnitReady(){
        long time = System.currentTimeMillis();
        do {
            pinpointDrive.update();
            GoBildaPinpointDriver.DeviceStatus status = pinpointDrive.getDeviceStatus();
            if (status == GoBildaPinpointDriver.DeviceStatus.READY) {
                Logger.message("pinpointDrive sensor is ready, time %d", System.currentTimeMillis() - time);
                break;
            } else if (System.currentTimeMillis() - time > 2000) {
                Logger.warning("pinpointDrive sensor not ready");
            }
        } while (true);
    }
}
