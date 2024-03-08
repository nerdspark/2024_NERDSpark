package frc.robot.util;

import frc.robot.config.RobotIdentity;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstantsTestbot;

public interface RobotConstants {

    public static TunerConstants getRobotConstants(RobotIdentity robot) {
        switch (robot) {
            case COMPETITION_ROBOT_2024:
                return new TunerConstants();
            case PRACTICE_ROBOT_2024:
                return new TunerConstantsTestbot();
            default:
                // Something went wrong if this branch is reached, by default we will return our Comp Bot
                return new TunerConstants();
        }
    }
}
