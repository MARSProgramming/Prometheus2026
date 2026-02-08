// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.commands.AimAndDriveCommand;
import frc.robot.commands.AutoRoutines;
import frc.robot.commands.ManualDriveCommand;
import frc.robot.constants.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;
import frc.robot.util.DrivetrainTelemetry;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed

    //ivate final Telemetry logger = new Telemetry(MaxSpeed);
    private final CommandXboxController pilot = new CommandXboxController(0);
    Swerve swervebase = new Swerve();
    DrivetrainTelemetry dttel = new DrivetrainTelemetry(swervebase);
    private final Limelight limelightStu = new Limelight("limelight-bob");
    private final AutoRoutines autoRoutines = new AutoRoutines(swervebase, limelightStu);


    public RobotContainer() {
        configureBindings();
        autoRoutines.configure(); // Handles autonomous command selection and configuration. Deprecates getAutonomousCommand() generated method
        // Initialize vision diagnostics
        visionDiagTable.getEntry("appliedCount").setNumber(appliedCount);
        visionDiagTable.getEntry("droppedCount").setNumber(droppedCount);
    }

    private void configureBindings() {  

        final ManualDriveCommand manualDriveCommand = new ManualDriveCommand(
            swervebase, 
            () -> -pilot.getLeftY(), 
            () -> -pilot.getLeftX(), 
            () -> -pilot.getRightX()
        );

        final AimAndDriveCommand aimAndDriveCommand = new AimAndDriveCommand(
            swervebase, 
            () -> -pilot.getLeftY(), 
            () -> -pilot.getLeftX());

        swervebase.setDefaultCommand(manualDriveCommand); // Handles teleoperated driving
        pilot.leftBumper().whileTrue(aimAndDriveCommand);
        
        pilot.rightBumper().onTrue(Commands.runOnce(
          () -> manualDriveCommand.setLockedHeading(
            Constants.Orientations.getClosestDiamond(swervebase.getState().Pose))
            )
        ); 

        limelightStu.setDefaultCommand(updateVisionCommand());
        pilot.back().onTrue(Commands.runOnce(() -> manualDriveCommand.seedFieldCentric())); // Re-seeds field-centric heading when 'back' button is pressed
    }

    // Update the robot's pose estimate using vision measurements from a Limelight
    private Command updateVisionCommand() {
        return limelightStu.run(() -> {
            final Pose2d currentRobotPose = swervebase.getState().Pose;
            final Optional<Limelight.Measurement> measurement = limelightStu.getMeasurement(currentRobotPose);
            measurement.ifPresent(m -> {
                // LimelightHelpers provides a wall-clock timestamp (seconds since epoch minus latency).
                // Convert that to WPILib FPGA timestamp domain so addVisionMeasurement receives
                // a Timer.getFPGATimestamp()-compatible timestamp.
                double measuredWallTime = m.poseEstimate.timestampSeconds; // seconds
                double nowWall = System.currentTimeMillis() / 1000.0;
                double nowFpga = Timer.getFPGATimestamp();
                double age = nowWall - measuredWallTime; // positive if measurement is in the past

                // Simple gating to avoid large/old/future corrections
                if (age < 0) {
                    visionDiagTable.getEntry("lastDroppedReason").setString("future_timestamp");
                    visionDiagTable.getEntry("droppedCount").setNumber(++droppedCount);
                    System.out.printf("Vision measurement from the future: age=%.3fs; dropping\n", age);
                    return;
                }
                if (age > 1.0) {
                    visionDiagTable.getEntry("lastDroppedReason").setString("too_old");
                    visionDiagTable.getEntry("droppedCount").setNumber(++droppedCount);
                    System.out.printf("Vision measurement too old: age=%.3fs; dropping\n", age);
                    return;
                }

                double measurementFpgaTimestamp = nowFpga - age;

                // Distance-based outlier rejection / clamping
                double dx = m.poseEstimate.pose.getX() - currentRobotPose.getX();
                double dy = m.poseEstimate.pose.getY() - currentRobotPose.getY();
                double dist = Math.hypot(dx, dy);

                final double CLAMP_THRESHOLD = 1.5; // distances above this will be clamped
                final double DROP_THRESHOLD = 3.0;  // distances above this will be dropped
                boolean clamped = false;
                Pose2d poseToApply = m.poseEstimate.pose;

                if (dist > DROP_THRESHOLD) {
                    visionDiagTable.getEntry("lastDroppedReason").setString("distance_too_far");
                    visionDiagTable.getEntry("droppedCount").setNumber(++droppedCount);
                    System.out.printf("Vision measurement too far: dist=%.3fm; dropping\n", dist);
                    return;
                } else if (dist > CLAMP_THRESHOLD) {
                    // Blend the measurement toward current pose so we only apply a bounded correction
                    double ratio = CLAMP_THRESHOLD / dist; // 0..1
                    double newX = currentRobotPose.getX() + dx * ratio;
                    double newY = currentRobotPose.getY() + dy * ratio;
                    // Limit heading change
                    double dthetaRaw = m.poseEstimate.pose.getRotation().getRadians() - currentRobotPose.getRotation().getRadians();
                    dthetaRaw = Math.atan2(Math.sin(dthetaRaw), Math.cos(dthetaRaw));
                    final double MAX_THETA_CHANGE = Math.toRadians(20.0);
                    double thetaChange = Math.max(-MAX_THETA_CHANGE, Math.min(MAX_THETA_CHANGE, dthetaRaw * ratio));
                    double newTheta = currentRobotPose.getRotation().getRadians() + thetaChange;
                    poseToApply = new Pose2d(newX, newY, new Rotation2d(newTheta));
                    clamped = true;
                    // recompute dx,dy,dist to reflect applied pose
                    dx = poseToApply.getX() - currentRobotPose.getX();
                    dy = poseToApply.getY() - currentRobotPose.getY();
                    dist = Math.hypot(dx, dy);
                }

                // Adaptive stddevs: start from measurement-reported stddevs (if present)
                double sx = 0.7, sy = 0.7, st = Math.toRadians(6.0);
                try {
                    Matrix<N3, N1> std = m.standardDeviations;
                    sx = std.get(0, 0);
                    sy = std.get(1, 0);
                    st = std.get(2, 0);
                } catch (Exception ex) {
                    // fallback to defaults if something unexpected
                }

                // Compute a simple confidence factor from tagCount, tagSpan and average tag distance.
                double tagCount = m.poseEstimate.tagCount;
                double tagSpan = m.poseEstimate.tagSpan; // meters
                double avgDist = m.poseEstimate.avgTagDist; // meters

                double tagCountFactor = Math.max(0.0, Math.min(1.0, tagCount / 3.0));
                double tagSpanFactor = Math.max(0.0, Math.min(1.0, tagSpan / 2.0));
                double distFactor = Math.max(0.0, Math.min(1.0, 1.0 - (avgDist / 4.0)));
                double confidence = 0.5 * tagCountFactor + 0.3 * tagSpanFactor + 0.2 * distFactor; // 0..1

                // Scale down stddev with higher confidence (more confident -> smaller stddev)
                double MIN_SX = 0.2, MIN_SY = 0.2, MIN_ST = Math.toRadians(2.0);
                double scale = 1.0 - 0.6 * confidence; // up to 60% reduction
                sx = Math.max(MIN_SX, sx * scale);
                sy = Math.max(MIN_SY, sy * scale);
                st = Math.max(MIN_ST, st * scale);

                visionDiagTable.getEntry("lastAppliedConfidence").setNumber(confidence);

                double dtheta = poseToApply.getRotation().getRadians() - currentRobotPose.getRotation().getRadians();
                dtheta = Math.atan2(Math.sin(dtheta), Math.cos(dtheta));

                double mahal = Math.sqrt((dx / sx) * (dx / sx) + (dy / sy) * (dy / sy) + (dtheta / st) * (dtheta / st));

                visionDiagTable.getEntry("lastMahalanobis").setNumber(mahal);

                final double MAHAL_THRESHOLD = 4.0; // tuneable: ~3-5
                if (mahal > MAHAL_THRESHOLD) {
                    visionDiagTable.getEntry("lastDroppedReason").setString("mahalanobis_reject");
                    visionDiagTable.getEntry("droppedCount").setNumber(++droppedCount);
                    System.out.printf("Vision measurement failed Mahalanobis gating: mah=%.3f; dropping\n", mahal);
                    return;
                }

                System.out.printf(
                    "Applying vision meas: tags=%d age=%.3fs dist=%.3fm mah=%.3f conf=%.3f clamped=%b fpgaTs=%.6f\n",
                    m.poseEstimate.tagCount,
                    age,
                    dist,
                    mahal,
                    confidence,
                    clamped,
                    measurementFpgaTimestamp
                );

                visionDiagTable.getEntry("lastAppliedAge").setNumber(age);
                visionDiagTable.getEntry("lastAppliedDist").setNumber(dist);
                visionDiagTable.getEntry("lastAppliedMahalanobis").setNumber(mahal);
                visionDiagTable.getEntry("lastAppliedClamped").setBoolean(clamped);
                visionDiagTable.getEntry("lastAppliedConfidence").setNumber(confidence);
                visionDiagTable.getEntry("appliedCount").setNumber(++appliedCount);

                // Create an adaptive stddev matrix to pass into the estimator
                Matrix<N3, N1> adaptiveStd = VecBuilder.fill(sx, sy, st);

                swervebase.addVisionMeasurement(
                    poseToApply,
                    measurementFpgaTimestamp,
                    adaptiveStd
                );
            });
        })
        .ignoringDisable(true);
    }

    // NetworkTables diagnostics for vision measurements
    private final NetworkTable visionDiagTable = NetworkTableInstance.getDefault().getTable("SmartDashboard/VisionDiagnostics");
    private int appliedCount = 0;
    private int droppedCount = 0;
}
