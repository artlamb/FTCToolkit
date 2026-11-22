package competition;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import common.Auto;
import common.Logger;

@Autonomous(name="Shoot Flower", group="Competition")

public class AutoShootFlower extends LinearOpMode {

    @Override
    public void runOpMode() {

        try {
            Auto.PathState[] order = {
                    Auto.PathState.START_HIVE_UP,
                    Auto.PathState.SHOOT_ONE,
                    Auto.PathState.FLOWER,
                    Auto.PathState.PARK
            };

            Auto auto = new Auto(this, Auto.Alliance.BLUE, order,29, 0);

            auto.runAuto();

        } catch (Exception e) {
            Logger.error(e, "Exception");
            throw e;
        }
    }
}
