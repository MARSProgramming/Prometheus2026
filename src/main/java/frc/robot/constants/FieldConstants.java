package frc.robot.constants;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class FieldConstants {
    public static class HubPoses {
            // Source: Purdue Ri3D 2026
            public static final Pose3d redHubPose = new Pose3d(Units.Inches.of(468.56), Units.Inches.of(158.32), Units.Inches.of(72.0), new Rotation3d());
            public static final Pose3d blueHubPose = new Pose3d(Units.Inches.of(152.56), Units.Inches.of(158.32),  Units.Inches.of(72.0), new Rotation3d());
        
        public static final Pose3d getHubPose() {
        Pose3d pose = DriverStation.getAlliance().equals(Optional.of(Alliance.Red)) ? redHubPose : blueHubPose;
        return pose;
        }
    }
    public static class AutoAim {

        // SOTM
        public static final double AUTO_AIM_MAX_DRIVETRAIN_VEL = 6.4; 
        // Compensation fudge factor for rotation
        public static final double ROTATION_VELOCITY_COMPENSATION_FACTOR = 0.1;

        //Turn to pose constants
        public static final double AUTO_AIM_MARGIN = 1.5;
        public static final double AUTO_AIM_SETPOINT_MARGIN = 5; // Degrees
        public static final double AUTO_AIM_MAX_VEL_SETPOINT = 1.5; // The maximum commanded rotational velocity in rad/s
        public static final double AUTO_AIM_MAX_ROT_VEL = 4.4;
        public static final double AUTO_AIM_KP = 0.1;
        public static final double AUTO_AIM_KD = 0; 

    }
}
