package common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import utils.Pose;

/**
 * This class handles all the driving related gamepad action. The class runs
 * on its own thread to avoid blocking.
 * <p><ul>
 * Actions:
 *      <br> left stick    - forward, back, strafe
 *      <br> right stick   - rotate
 *      <br> left trigger  - turn left by 2 degrees
 *      <br> right trigger - turn left by 2 degrees
 *      <br> right bumper  - align robot using distance sensors
 *      <br> a, b, x, y    - move to preset pose
 *      <br> left bumper + [a b x y] - set current pose to button
 *      <br> back - emergency stop
 *      <br>
 *
 * </ul></p>
 */
public class DriveGamepad extends Thread {

    private final double MAX_WAYPOINT_SPEED = 0.5;

    public enum PoseButton { A, B, X, Y}
    private final int poseCount = PoseButton.values().length;
    private final Pose[] poses = new Pose[poseCount];

    LinearOpMode opMode;
    DriveControl driveControl;

    public DriveGamepad(LinearOpMode opMode, DriveControl driveControl) {

        this.opMode = opMode;
        this.driveControl = driveControl;
    }

    /**
     * Control the gamepad driving actions on a separate thread to avoid blocking
     */
    public void run() {

        Logger.message("drive gamepad thread started for %s", this.getName());

        while (!opMode.isStarted()) {
            Thread.yield();
        }

        while (opMode.opModeIsActive()) {
            driveWithJoysticks();
        }

        Logger.message("drive gamepad control thread stopped");
    }

    /**
     * Handle all the gamepad driving actions.
     */
    private void driveWithJoysticks() {

        boolean moving = false;

        while (opMode.opModeIsActive()) {

            Gamepad gamepad = opMode.gamepad1;

            if (gamepad.atRest()) {
                Thread.yield();
            }

            if (gamepad.back) {
                driveControl.emergencyStop();
                break;
            }

            if (gamepad.a) {
                moveToPose(PoseButton.A);
                while (gamepad.a) Thread.yield();
            } else if (gamepad.b) {
                moveToPose(PoseButton.B);
                while (gamepad.b) Thread.yield();
            } else if (gamepad.x) {
                moveToPose(PoseButton.X);
                while (gamepad.x) Thread.yield();
            } else if (gamepad.y) {
                moveToPose(PoseButton.Y);
                while (gamepad.y) Thread.yield();
            }

            if (gamepad.right_bumper) {
                driveControl.alignInCorner();
                while (gamepad.right_bumper)  Thread.yield();
            }

            if (gamepad.left_bumper) {
                while (gamepad.left_bumper)
                    if (gamepad.a) {
                        setToCurrentPosition(PoseButton.A);
                        while (gamepad.a) Thread.yield();
                    } else if (gamepad.b) {
                        setToCurrentPosition(PoseButton.B);
                        while (gamepad.b) Thread.yield();
                    } else if (gamepad.x) {
                        setToCurrentPosition(PoseButton.X);
                        while (gamepad.x) Thread.yield();
                    } else if (gamepad.y) {
                        setToCurrentPosition(PoseButton.Y);
                        while (gamepad.y) Thread.yield();
                    }
                while (gamepad.left_bumper) Thread.yield();
            }

            if (gamepad.left_trigger > 0) {
                driveControl.turnBy(2, 1000);
                while (gamepad.left_trigger > 0) Thread.yield();
            }

            if (gamepad.right_trigger > 0) {
                driveControl.turnBy(-2, 1000);
                while (gamepad.right_trigger > 0) Thread.yield();
            }

            // Left stick to go forward, back and strafe. Right stick to rotate.
            double x = gamepad.left_stick_x;
            double y = -gamepad.left_stick_y;
            double x2 = gamepad.right_stick_x;
            double noise = 0.01;

            // Is either stick being used?
            if (Math.abs(x) > noise || Math.abs(y) > noise || Math.abs(x2) > noise ) {

                if (! moving) {
                    driveControl.startMoving();
                    moving = true;
                }
                driveControl.moveWithJoystick(x, y, x2);

            } else if (moving) {
                moving = false;
                driveControl.stopMoving();
            }
        }
    }

    /**
     * Associate a pose with a gamepad button. When the specified button is pressed
     * the robot moves to the specified pose
     *
     * @param button  gamepad button [a b x y]
     * @param pose    pose
     */
    public void setPosePosition (PoseButton button, Pose pose) {
        int index = button.ordinal();
        poses[index] = pose;
    }

    /**
     * Associate the current robot pose with a gamepad button. When the specified button is pressed
     * the robot moves to the current pose.
     *
     * @param button  gamepad button [a b x y]
     */
    private void setToCurrentPosition(PoseButton button) {
        Pose pose = driveControl.getPose();
        setPosePosition(button, pose);
    }

    /**
     * Move to the pose associated with the specified button.
     *
     * @param button [a b x y]
     */
    private void moveToPose(PoseButton button) {
        int index = button.ordinal();
        Pose pose =  poses[index];
        if (pose != null) {
            double x = pose.getX();
            double y = pose.getY();
            double heading = Math.toDegrees(pose.getHeading());

            Logger.message("%s  move to (%.1f, %.1f) heading: %.0f", button, x, y, heading);
            driveControl.moveToCoordinate(x, y, heading, MAX_WAYPOINT_SPEED, 4000 );
        }
    }
}



