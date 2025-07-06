/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package test;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.ArrayList;

import common.Robot;
import utils.Pose;

import common.DriveControl;
import common.DriveGamepad;
import common.Logger;
import utils.PoseData;

@TeleOp(name=" Path Test", group="Test")
@SuppressLint("DefaultLocale")
@Config

public class PathTest extends LinearOpMode {
    public static Boolean AUTO_DRIVE = true;

    public enum PathState { START, WAYPOINT_1, WAYPOINT_2, PARK }
    public static PoseData[] WAYPOINTS = {
            new PoseData( 0, 0, 0, PathState.START.toString()),
            new PoseData(20, 0, 0, PathState.WAYPOINT_1.toString()),
            new PoseData( 0, 0, 0, PathState.WAYPOINT_2.toString()),
            new PoseData( 0, 0, 0, PathState.PARK.toString())
    };

    private final ArrayList<Pose> poses = new ArrayList<>();
    private final ArrayList<String> names = new ArrayList<>();

    private DriveControl driveControl;
    DriveGamepad driveGamepad;

    @Override
    public void runOpMode() {

        try {
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

            initialize();
            addPaths();
            driveControl.start();

            telemetry.addLine("Press start");
            telemetry.update();
            waitForStart();

            if (AUTO_DRIVE) {
                driveGamepad.start();
                followPaths();
            } else {
                manualDrive();
            }

        } catch (Exception e) {
            Logger.error(e, "Exception");
            throw e;
        }
    }

    private void initialize() {
        Robot  robot = new Robot(this);
        driveControl = robot.getDriveControl();
        driveGamepad = new DriveGamepad(this, driveControl);
    }

    private Pose createPose(double x, double y, double heading) {
        return new Pose(x, y, Math.toRadians(heading));
    }

    private void addPaths() {
        double x = 0, y = 0, h = 0;
        int waypoint = 0;
        for (PoseData data: WAYPOINTS) {
            // The first waypoint is the starting position
            if (waypoint > 0) {
                // skip duplicate waypoints
                if (x != data.x || y != data.y || h != data.h) {
                    Pose pose = createPose(data.x, data.y, data.h);
                    String name = data.desc;
                    poses.add(pose);
                    names.add(name);
                }
            }
            x = data.x;
            y = data.y;
            h = data.h;
            waypoint++;
        }
    }

    private void followPaths() {

        long start = System.currentTimeMillis();

        Pose pose = poses.get(0);
        driveControl.setPose(createPose(pose.getX(), pose.getY(), pose.getHeading()));

        for (int i = 1; i < poses.size(); i++) {
            pose = poses.get(i);
            Logger.info("Moving to %s  x %5.1f  y %5.1f  heading %6.1f", names.get(i), pose.getX(), pose.getY(), pose.getHeading());
            //driveControl.moveToPose(pose, 4000);
            while (driveControl.isBusy() && opModeIsActive()) {
                sleep(1);
            }
            sleep(100);
            if (! opModeIsActive())
                break;
        }

        Logger.message(String.format("time: %,d milliseconds", System.currentTimeMillis() - start));
    }

    private void manualDrive() {

        double timeout = 5000;

        Pose pose = poses.get(0);
        driveControl.setPose(pose);

        while (opModeIsActive()) {

            if (gamepad1.a) {
                pose = poses.get(0);
                driveControl.moveToPose(pose, timeout);
                while (gamepad1.a) sleep(10);
            }

            if (gamepad1.b) {
                pose = poses.get(1);
                driveControl.moveToPose(pose, timeout);
                while (gamepad1.b) sleep(10);
            }

            if (gamepad1.x) {
                pose = poses.get(2);
                driveControl.moveToPose(pose, timeout);
                while (gamepad1.x) sleep(10);
            }

            if (gamepad1.y) {
                pose = poses.get(3);
                driveControl.moveToPose(pose, timeout);
                while (gamepad1.y) sleep(10);
            }

            waitUntilNotMoving();
            displayPose();
        }
    }

    private void waitUntilNotMoving() {
        while (driveControl.isBusy() && opModeIsActive()) {
            sleep(1);
        }
        sleep(100);
    }

    private void displayPose() {

        Pose pose = driveControl.getPose();

        String str1 = String.format("x %5.1f  y %5.1f  heading %5.1f", pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));

        telemetry.addData("pose", str1);
        telemetry.update();

        Logger.message("pose: %s", str1);
    }
}
