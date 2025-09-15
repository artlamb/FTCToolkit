package test;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import java.util.ArrayList;
import java.util.List;

import common.Logger;
import common.Robot;

@TeleOp(name="Dashboard Test", group="Test")
@SuppressLint("DefaultLocale")
@Disabled

public class CurrentTest extends LinearOpMode {

    Robot robot;

    @Override
    public void runOpMode() {
        try {
            robot = new Robot(this);
            waitForStart();

            motorTest();

            while (opModeIsActive()) {
                sleep(500);
            }

        } catch (Exception e) {
            Logger.error(e, "Error");
        }
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
}
