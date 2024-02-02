// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team9062.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team9062.robot.Commands.TeleopArm;
import frc.team9062.robot.Commands.TeleopDrive;
import frc.team9062.robot.Subsystems.Arm;
import frc.team9062.robot.Subsystems.SwerveSubsystem;
import frc.team9062.robot.Util.SystemState.VERBOSITY_LEVEL;

public class RobotContainer {
  private SwerveSubsystem swerve;
  private Arm arm;
  public SendableChooser<VERBOSITY_LEVEL> verbosityChooser = new SendableChooser<>();
  public SendableChooser<String> autoChooser = new SendableChooser<>();

  public RobotContainer() {
    swerve = SwerveSubsystem.getInstance();
    arm = Arm.getInstance();

    configureBindings();

    verbosityChooser.setDefaultOption("LOW", VERBOSITY_LEVEL.LOW);
    verbosityChooser.addOption("HIGH", VERBOSITY_LEVEL.HIGH);
    verbosityChooser.addOption("COMP", VERBOSITY_LEVEL.COMP);

    autoChooser.addOption("6 Piece", "6 Piece");
    autoChooser.addOption("5 Piece", "5 Piece");
    autoChooser.addOption("4 Piece Inside", "4 Piece Inside");
    autoChooser.addOption("4 Piece Inside - Fast", "4 Piece Inside - Fast");
    autoChooser.addOption("Test Auto", "Test Auto");

    //autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("VERBOSITY", verbosityChooser);
    SmartDashboard.putData("AUTO", autoChooser);

    swerve.setDefaultCommand(
      new TeleopDrive()
    );

    arm.setDefaultCommand(
      new TeleopArm()
    );
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    Pose2d pose = PathPlannerAuto.getStaringPoseFromAutoFile(autoChooser.getSelected());

    return new SequentialCommandGroup(
      new InstantCommand(() -> swerve.resetOdom(pose), swerve),
      AutoBuilder.buildAuto(autoChooser.getSelected())
    );
  }
}