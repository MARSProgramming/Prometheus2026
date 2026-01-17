// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Drive;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed

    //ivate final Telemetry logger = new Telemetry(MaxSpeed);
    private final CommandXboxController pilot = new CommandXboxController(0);
    Drive swervebase = new Drive();
    DrivetrainTelemetry dttel = new DrivetrainTelemetry(swervebase);


    public RobotContainer() {
        configureBindings();

    }

    private void configureBindings() {
        swervebase.setDefaultCommand(swervebase.teleopDrive(pilot));


        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        
        // Reset the field-centric heading on left bumper press.
        pilot.leftBumper().onTrue(swervebase.seedCentric());

    }

    public Command getAutonomousCommand() {
        // Simple drive forward auton
        return Commands.none();
    }
    /**
     * Applies a deadband to inputs.
     * @param value The input value to apply the deadband to.
     * @param deadband The size of the deadband.
     */


}
