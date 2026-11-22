package competition;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import common.Auto;
import common.Logger;

@Autonomous(name="Park", group="Competition")

public class AutoPark extends LinearOpMode {

    @Override
    public void runOpMode() {

        try {
            Auto.PathState[] order = {
                    Auto.PathState.START_CONNOR,
                    Auto.PathState.PARK_AUDIENCE
            };

            Auto auto = new Auto(this, Auto.Alliance.BLUE, order,29, 0);

            auto.runAuto();

        } catch (Exception e) {
            Logger.error(e, "Exception");
            throw e;
        }
    }
}
