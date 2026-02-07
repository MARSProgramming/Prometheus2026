package frc.robot.constants;

import java.util.ArrayList;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Swerve;

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

        
        public static Rotation2d getDirectionToHub(Swerve sw) {
        final Translation2d hubPosition = HubPoses.hubPosition();
        final Translation2d robotPosition = sw.getState().Pose.getTranslation();
        final Rotation2d hubDirectionInBlueAlliancePerspective = hubPosition.minus(robotPosition).getAngle();
        final Rotation2d hubDirectionInOperatorPerspective = hubDirectionInBlueAlliancePerspective.rotateBy(sw.getOperatorForwardDirection());
        return hubDirectionInOperatorPerspective;
        }
    }

    public static class HubPoses {
            // Source: Purdue Ri3D 2026
    public static Translation2d hubPosition() {
        final Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
            return new Translation2d(Units.Inches.of(182.105), Units.Inches.of(158.845));
        }
        return new Translation2d(Units.Inches.of(469.115), Units.Inches.of(158.845));
    }

    public static int[] getValidTagIDs() {
        final Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
            return new int[]{17,18,19,20,21,22,23,24,25,26,27,28,29,30}; // Valid blue tags
        } else {
            return new int[]{1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16}; // Red alliance sees these AprilTags
        }
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
