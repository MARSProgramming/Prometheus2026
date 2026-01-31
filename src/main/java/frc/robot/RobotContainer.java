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
    private final Limelight limelightStu = new Limelight("limelight-stuart");
    private final AutoRoutines autoRoutines = new AutoRoutines(swervebase, limelightStu);


    public RobotContainer() {
        configureBindings();
        autoRoutines.configure(); // Handles autonomous command selection and configuration. Deprecates getAutonomousCommand() generated method
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
                swervebase.addVisionMeasurement(
                    m.poseEstimate.pose, 
                    m.poseEstimate.timestampSeconds,
                    m.standardDeviations
                );
            });
        })
        .ignoringDisable(true);
    }
}
