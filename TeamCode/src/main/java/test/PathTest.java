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
import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.util.ArrayList;
import java.util.List;

import common.Navigate;
import common.Robot;
import utils.Pose;

import common.DriveControl;
import common.DriveGamepad;
import common.Logger;
import utils.PoseData;
import utils.Waypoint;

@TeleOp(name=" Path Test", group="Test")
@SuppressLint("DefaultLocale")
@Config

public class PathTest extends LinearOpMode {
    public enum Mode {AUTO_POSE, AUTO_PATHS, MANUAL, GAMEPAD}

    public static Mode MODE = Mode.AUTO_PATHS;
    public static boolean READ_POSES = true;
    public static boolean WRITE_POSES = true;

    public static volatile PoseData[] waypoints = {
            new PoseData(0,  0,  0, Waypoint.START),
            new PoseData(20, 0,  0, Waypoint.WAYPOINT_1),
            new PoseData(25, 3,  0, Waypoint.WAYPOINT_2),
            new PoseData(30, 10, 0, Waypoint.WAYPOINT_2),
            new PoseData(0,  0,  0, Waypoint.PARK)
    };

    private final ArrayList<Pose> poses = new ArrayList<>();
    private final ArrayList<String> names = new ArrayList<>();

    private Robot robot;
    private DriveControl driveControl;
    private DriveGamepad driveGamepad;
    private Navigate navigate;

    @Override
    public void runOpMode() {

        try {

            initialize();

            if (READ_POSES)
                readPoses();

            switch (MODE) {
                case AUTO_POSE:
                    addPoses();
                    break;
                case AUTO_PATHS:
                    addPaths();
                    break;
                case GAMEPAD:
                    driveGamepad.start();
                    break;
            }

            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            telemetry.addLine("Press start");
            telemetry.update();
            waitForStart();

            switch (MODE) {
                case AUTO_POSE:
                    moveToPoses();
                    break;
                case AUTO_PATHS:
                    navigate.displayPaths();
                    //followPaths();
                    break;
                case MANUAL:
                    manualDrive();
                    break;
                case GAMEPAD:
                    gamepadDrive();
                    break;
            }

            if (WRITE_POSES)
                writePoses();

            while (opModeIsActive()) sleep(10);

        } catch (Exception e) {
            Logger.error(e, "Exception");
            throw e;
        }
    }

    private void initialize() {
        robot = new Robot(this);
        driveControl = robot.getDriveControl();
        driveGamepad = new DriveGamepad(this, driveControl);
        navigate = new Navigate(this, driveControl);

    }

    private void motorTest() {
        String[] names = {
                common.Config.LEFT_FRONT,
                common.Config.RIGHT_FRONT,
                common.Config.LEFT_BACK,
                common.Config.RIGHT_BACK};
        List<DcMotorEx> motors = new ArrayList<>();

        for (String name : names) {
            Logger.message("name %s", name);
            DcMotorEx motor;
            motor = hardwareMap.get(DcMotorEx.class, name);
            motors.add(motor);
        }
        for (DcMotorEx motor : motors) {
            int position = motor.getCurrentPosition();
            Logger.message("position %d", position);
            motor.setVelocity(robot.drive.getMaxVelocity() * 0.6);
        }
        long start = System.currentTimeMillis();
        do {
            Thread.yield();
        } while (System.currentTimeMillis() < start + 2000);

        for (DcMotorEx motor : motors) {
            motor.setVelocity(0);
        }
    }

    private Pose createPose(double x, double y, double heading) {
        return new Pose(x, y, Math.toRadians(heading));
    }

    private void addPoses() {
        double x = 0, y = 0, h = 0;
        int waypoint = 0;
        for (PoseData data: waypoints) {
            // The first waypoint is the starting position
            if (waypoint > 0) {
                // skip duplicate waypoints
                if (x != data.x || y != data.y || h != data.h) {
                    Pose pose = createPose(data.x, data.y, data.h);
                    String name = data.desc.name();
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

    private void moveToPoses() {

        long start = System.currentTimeMillis();

        Pose pose = poses.get(0);
        driveControl.setPose(createPose(pose.getX(), pose.getY(), pose.getHeading()));

        for (int i = 1; i < poses.size(); i++) {
            pose = poses.get(i);
            Logger.info("Moving to %s  x %5.1f  y %5.1f  heading %6.1f", names.get(i), pose.getX(), pose.getY(), pose.getHeading());
            driveControl.moveToPose(pose, 4000);
            while (driveControl.isBusy() && opModeIsActive()) {
                sleep(1);
            }
            sleep(100);
            if (! opModeIsActive())
                break;
        }

        Logger.message(String.format("time: %,d milliseconds", System.currentTimeMillis() - start));
    }

    private void addPaths() {

        double x = 0, y = 0, h = 0;
        int waypoint = 0;
        for (PoseData data: waypoints) {
            Pose pose = createPose(data.x, data.y, data.h);
            // The first waypoint is the starting position
            if (waypoint == 0) {
                navigate.addPath(data.desc.name(), pose);

            } else if (x != data.x || y != data.y || h != data.h) {   // skip duplicate waypoints
                if (navigate.pathExists(data.desc.name())) {
                    navigate.appendPose(data.desc.name(), pose);
                } else {
                    navigate.addPath(data.desc.name(), pose);
                }
            }
            x = data.x;
            y = data.y;
            h = data.h;
            waypoint++;
        }
    }

    private void followPaths() {

        navigate.setStartingPose(0);

        for (int index = 1; index < navigate.numberOfPaths(); index++) {
            navigate.followPath(index);
            waitUntilNotMoving();
        }
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

    private void gamepadDrive() {
        while (opModeIsActive()) {
            Thread.yield();
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

    private void writePoses ()  {

        try {
            String filePath = String.format("%s/%s/%s", AppUtil.FIRST_FOLDER.getAbsolutePath(), "settings", "PathTest.txt");
            String backPath = String.format("%s/%s/%s", AppUtil.FIRST_FOLDER.getAbsolutePath(), "settings", "PathTest_Backup.txt");

            File file = new File(filePath);
            Logger.message("%s exist is %b", filePath, file.exists());
            if (file.exists()) {
                boolean renamed = file.renameTo(new File(backPath));
                if (renamed)
                    Logger.message("%s renamed to %s", filePath, backPath);
            }

            Gson gson = new GsonBuilder().setPrettyPrinting().create();
            FileWriter f = new FileWriter(filePath, false);

            gson.toJson(waypoints, f);
            f.flush();
            f.close();

        } catch (Exception e) {
            Logger.error(e, "Error writing settings");
        }
    }

    private void readPoses() {

        String filePath = String.format("%s/%s/%s", AppUtil.FIRST_FOLDER.getAbsolutePath(), "settings", "PathTest.txt");

        try {
            BufferedReader br = new BufferedReader(new FileReader(filePath));
            //Type listType = new TypeToken<List<PoseData>>(){}.getType(); // For a List
            waypoints = new Gson().fromJson(br, PoseData[].class);
        } catch (Exception e) {
            Logger.error(e, "Error reading settings");
        }
    }


}
