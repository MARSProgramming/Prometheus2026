package frc.robot.subsystems;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.generated.TunerConstants;
import limelight.Limelight;
import limelight.networktables.LimelightPoseEstimator;
import limelight.networktables.LimelightSettings.ImuMode;
import limelight.networktables.LimelightSettings.LEDMode;
import limelight.networktables.PoseEstimate;
import limelight.networktables.LimelightPoseEstimator.EstimationMode;

public class Drive extends CommandSwerveDrivetrain {
    private final double stickDeadband = 0.1; // configurable deadband for controller
    private final double maxAngularRate = Units.RotationsPerSecond.of(0.75).in(Units.RadiansPerSecond); // configure the maximum rotational velocity in teleoperated mode.
    private final double maxSpeed = TunerConstants.kSpeedAt12Volts.in(Units.MetersPerSecond); // configure the maximum speed in teleoperated mode.
    
    private final SwerveRequest.FieldCentric teleopRequest = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    
    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();
    private Limelight limelight_bob = new Limelight("limelight-bob");
    private LimelightPoseEstimator bobEstimator = new LimelightPoseEstimator(limelight_bob, EstimationMode.MEGATAG2);


    // Instantiate a new instance of the Drive subsystem.
    public Drive() {
        super(TunerConstants.DrivetrainConstants, TunerConstants.FrontLeft, TunerConstants.FrontRight, TunerConstants.BackLeft, TunerConstants.BackRight);
        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                () -> getState().Pose,   // Supplier of current robot pose
                this::resetPose,         // Consumer for seeding pose against auto
                () -> getState().Speeds, // Supplier of current robot speeds
                // Consumer of ChassisSpeeds and feedforwards to drive the robot
                (speeds, feedforwards) -> setControl(
                    m_pathApplyRobotSpeeds.withSpeeds(ChassisSpeeds.discretize(speeds, 0.020))
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                ),
                new PPHolonomicDriveController(
                    // PID constants for translation
                    new PIDConstants(10, 0, 0),
                    // PID constants for rotation
                    new PIDConstants(7, 0, 0)
                ),
                config, 
                // Assume the path needs to be flipped for Red vs Blue, this is normally the
                // case
                () -> {
                        var alliance = DriverStation.getAlliance();
                        if (alliance.isPresent()) {
                            return alliance.get() == DriverStation.Alliance.Red;
                        }
                        return false;
                    },
                    this
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
        }

        // Instantiate first limelight.
        //TODO: Test if it can handle disconnects. Instantiate a nonexistent limelight to check.

        limelight_bob = new Limelight("limelight-bob");
        
        limelight_bob.getSettings()
          .withLimelightLEDMode(LEDMode.PipelineControl)
          .withCameraOffset(new Pose3d(
              Units.Inches.of(0).in(Units.Meters),
              Units.Inches.of(0).in(Units.Meters),
              Units.Inches.of(0).in(Units.Meters),
              new Rotation3d(0, 0, 0))) // report in radians if necessary
          .withImuMode(ImuMode.InternalImuMT1Assist)
          .withImuAssistAlpha(0.01)
          .save();

        RobotModeTriggers.disabled().onTrue(Commands.runOnce(() -> {
        System.out.println("Neutralizing IMU Assist");

        limelight_bob.getSettings()
            // .withImuMode(ImuMode.InternalImuMT1Assist)
            .withImuAssistAlpha(0.001)
            .save();
      }).ignoringDisable(true));
    }

    /**
     * Applies a teleoperated drive request based on controller input.
     * @param controller the controller to pull for the inputs.
     */

    public Command teleopDrive(CommandXboxController controller) {
        return super.applyRequest(() ->
            teleopRequest.withVelocityX(deadband(-controller.getLeftY(), 0.1)  * maxSpeed) // Drive
                                                                                                             // forward
                                                                                                             // with
                                                                                                             // negative
                                                                                                             // Y (up)
            .withVelocityY(deadband(-controller.getLeftX(), 0.1) * maxSpeed) // Drive left with negative X (left)
            .withRotationalRate(deadband(-controller.getRightX(), 0.1) * maxAngularRate) // Drive counterclockwise with
                                                                                    // negative X (left)
        );
    }

    public Command seedCentric() {
        return Commands.runOnce(() -> {
            this.seedFieldCentric();
        });
    }


    @Override
    public void periodic() {
        super.periodic();
        Optional<PoseEstimate> visionEstimate = bobEstimator.getPoseEstimate();

      // If the pose is present
      visionEstimate.ifPresent((PoseEstimate poseEstimate) -> {
        if (poseEstimate.tagCount > 0) {
          SmartDashboard.putNumber("Limelight/Megatag2Count", poseEstimate.tagCount);          

          Pose3d redHub = new Pose3d(
              Units.Meters.of(11.902), // configure
              Units.Meters.of(4.031), // configure 
              Units.Meters.of(0.0), // configure 
              new Rotation3d(0, 0, 0));


          // A test value to get our current distance to the hub.
          double distanceToHub = getState().Pose.minus(redHub.toPose2d()).getTranslation().getNorm();

          SmartDashboard.putNumber("FieldSimulation/hubDiff", distanceToHub);

          // Add it to the pose estimator.
          super.addVisionMeasurement(
              poseEstimate.pose.toPose2d(),
              poseEstimate.timestampSeconds);

          super.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));

        }
      });
    }

    
    private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);

      }
    } else {
      return 0.0;
    }
  }
}

