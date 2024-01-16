// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team9062.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.team9062.robot.Commands.TeleopDrive;
import frc.team9062.robot.Subsystems.SwerveSubsystem;
import frc.team9062.robot.Util.RobotState.VERBOSITY_LEVEL;

public class RobotContainer {
  private SwerveSubsystem swerve;
  public SendableChooser<VERBOSITY_LEVEL> verbosityChooser = new SendableChooser<>();

  public RobotContainer() {
    swerve = SwerveSubsystem.getInstance();

    configureBindings();

    verbosityChooser.setDefaultOption("LOW", VERBOSITY_LEVEL.LOW);
    verbosityChooser.addOption("HIGH", VERBOSITY_LEVEL.HIGH);
    verbosityChooser.addOption("COMP", VERBOSITY_LEVEL.COMP);

    swerve.setDefaultCommand(
      new TeleopDrive()
    );
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
