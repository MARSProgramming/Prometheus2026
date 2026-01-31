package frc.robot.constants;

import java.util.ArrayList;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.generated.TunerConstants;

public class Constants {

        public static class Orientations {
        public static ArrayList<Rotation2d> legalOrientations = new ArrayList<Rotation2d>();
        static {
            legalOrientations.add(new Rotation2d(Units.Degrees.of(45)));
            legalOrientations.add(new Rotation2d(Units.Degrees.of(135)));
            legalOrientations.add(new Rotation2d(Units.Degrees.of(225)));
            legalOrientations.add(new Rotation2d(Units.Degrees.of(315)));
        }

        /*
         * Return the closest diamond orientation for the robot
         */

        public static Rotation2d getClosestDiamond(Pose2d robotPose) {
            Rotation2d currRot = robotPose.getRotation();
            Rotation2d closest = legalOrientations.get(0);
            double minError = Math.abs(currRot.minus(closest).getRadians());

            for (Rotation2d candidate : legalOrientations) {
                double error = Math.abs(currRot.minus(candidate).getRadians());
                if (error < minError) {
                minError = error;
                closest = candidate;
                }
            }

            return closest;
        }
    }

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

        public static class Driving {
        public static final LinearVelocity kMaxSpeed = TunerConstants.kSpeedAt12Volts;
        public static final AngularVelocity kMaxRotationalRate = Units.RotationsPerSecond.of(1);
        public static final AngularVelocity kPIDRotationDeadband = kMaxRotationalRate.times(0.005);
    }

    public static class KrakenX60 {
        public static final AngularVelocity kFreeSpeed = Units.RPM.of(6000);
    }
}
