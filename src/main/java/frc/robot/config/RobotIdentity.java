package frc.robot.config;

import static frc.robot.util.MacAddress.*;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.MacAddress;

public enum RobotIdentity {
    COMPETITION_ROBOT_2024,
    PRACTICE_ROBOT_2024,
    SIMULATION;

    public static RobotIdentity getIdentity() {
        if (Robot.isReal()) {
            String mac = getMACAddress();
            if (!mac.equals("")) {
                if (mac.equals(MacAddress.CRAZY_ARM_MAC) || mac.equals(SECONDARY_ROBOT_ONE_MAC)) {
                    return COMPETITION_ROBOT_2024;
                } else if (mac.equals(MacAddress.SWERVEBOT_2_MAC)) {
                    return PRACTICE_ROBOT_2024;
                }
            }
            if (Constants.compRobot == COMPETITION_ROBOT_2024) {
                return COMPETITION_ROBOT_2024;
            } else {
                return PRACTICE_ROBOT_2024;
            }
        } else {
            return SIMULATION;
        }
    }
}
/*public enum RobotIdentity {
   COMPETITION_ROBOT_2024,
   PRACTICE_ROBOT_2024,
   SIMULATION,

   public static RobotIdentity getIdentity() {
       if (Robot.isReal()) {
           String mac = getMACAddress();
           if (!mac.equals("")) {
               if (mac.equals(MacAddress.ROBOT_ONE_MAC) || mac.equals(SECONDARY_ROBOT_ONE_MAC)) {
                   return COMPETITION_ROBOT_2024;
               } else {
                   return PRACTICE_ROBOT_2024;
               }

       } else {
           return SIMULATION;
       }
   }
}*/
