// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team9062.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.team9062.robot.Commands.RumbleCommand;
import frc.team9062.robot.Commands.TeleopDrive;
import frc.team9062.robot.Commands.feed;
import frc.team9062.robot.Commands.manualArm;
import frc.team9062.robot.Commands.Auto.Intake;
import frc.team9062.robot.Subsystems.Arm;
import frc.team9062.robot.Subsystems.LEDSubsystem;
import frc.team9062.robot.Subsystems.Rollers;
import frc.team9062.robot.Subsystems.Drive.SwerveSubsystem;
import frc.team9062.robot.Subsystems.LEDSubsystem.LED_STATE;
import frc.team9062.robot.Subsystems.Rollers.INTAKE_STATE;
import frc.team9062.robot.Subsystems.Arm.ARM_STATE;
import frc.team9062.robot.Util.gamepadConfigs;
import frc.team9062.robot.Util.SystemState.VERBOSITY_LEVEL;

public class RobotContainer {
  private SwerveSubsystem swerve;
  private Arm arm;
  private Rollers rollers;
  private LEDSubsystem led;
  private SendableChooser<VERBOSITY_LEVEL> verbosityChooser = new SendableChooser<>();
  private SendableChooser<String> autoChooser = new SendableChooser<>();
  private CommandXboxController operator;
  //private CommandXboxController driver;

  public RobotContainer() {
    arm = Arm.getInstance();
    rollers = Rollers.getInstance();
    led = LEDSubsystem.getInstance();
    swerve = SwerveSubsystem.getInstance();

    NamedCommands.registerCommand("Intake", new Intake());
    NamedCommands.registerCommand(
      "Score1", 
      new ParallelDeadlineGroup(
        new WaitCommand(1.2),
        new SequentialCommandGroup(
          new InstantCommand(() -> rollers.setIntakeState(INTAKE_STATE.IDLE)),
          new InstantCommand(() -> arm.setCustomSetpoint(0.46), arm),
          //new InstantCommand(() -> arm.setArmState(ARM_STATE.AIM), arm),
          new ParallelCommandGroup(
            new RepeatCommand(
              new InstantCommand(() -> rollers.setShooterVelocity(60))
            ),
            new SequentialCommandGroup(
              new WaitCommand(0.6),
              new feed()
            )
          )
        )
      )
    );
    NamedCommands.registerCommand(
      "Score2", 
      new ParallelDeadlineGroup(
        new WaitCommand(1.2),
        new SequentialCommandGroup(
          new InstantCommand(() -> rollers.setIntakeState(INTAKE_STATE.IDLE)),
          new InstantCommand(() -> arm.setCustomSetpoint(0.465), arm),
          //new InstantCommand(() -> arm.setArmState(ARM_STATE.AIM), arm),
          new ParallelCommandGroup(
            new RepeatCommand(
              new InstantCommand(() -> rollers.setShooterVelocity(65))
            ),
            new SequentialCommandGroup(
              new WaitCommand(0.7),
              new feed()
            )
          )
        )
        )
    );
    NamedCommands.registerCommand(
      "Score3", 
      new ParallelDeadlineGroup(
        new WaitCommand(1.5),
        new SequentialCommandGroup(
          new InstantCommand(() -> rollers.setIntakeState(INTAKE_STATE.IDLE)),
          new InstantCommand(() -> arm.setCustomSetpoint(0.455), arm),
          //new InstantCommand(() -> arm.setArmState(ARM_STATE.AIM), arm),
          new ParallelCommandGroup(
            new RepeatCommand(
              new InstantCommand(() -> rollers.setShooterVelocity(65))
            ),
            new SequentialCommandGroup(
              new WaitCommand(1),
              new feed()
            )
          )
        )
        )
    );
    NamedCommands.registerCommand(
      "ScoreSubwoofer", 
      new ParallelDeadlineGroup(
        new WaitCommand(1.25),
        new SequentialCommandGroup(
          new InstantCommand(() -> arm.setArmState(ARM_STATE.LOW), arm),
          new ParallelCommandGroup(
            new RepeatCommand(
              new InstantCommand(() -> rollers.setShooterVelocity(40))
            ),
            new SequentialCommandGroup(
              new WaitCommand(0.8),
              new feed()
            )
          )
        )
      )
    );

    NamedCommands.registerCommand(
      "ScoreSubwooferF", 
      new ParallelDeadlineGroup(
        new WaitCommand(0.5),
        new SequentialCommandGroup(
          new InstantCommand(() -> arm.setArmState(ARM_STATE.LOW), arm),
          new ParallelCommandGroup(
            new RepeatCommand(
              new InstantCommand(() -> rollers.setShooterVelocity(40))
            ),
            new SequentialCommandGroup(
              new WaitCommand(0.1),
              new feed()
            )
          )
        )
      )
    );

    NamedCommands.registerCommand(
      "Shoot Far",
      new ParallelCommandGroup(
        new WaitCommand(1),
        new SequentialCommandGroup(
          new InstantCommand(() -> arm.setArmState(ARM_STATE.AIM), arm),
          new ParallelCommandGroup(
            new RepeatCommand(
              new InstantCommand(() -> rollers.setShooterVelocity(80))
            ),
            new SequentialCommandGroup(
              new WaitCommand(0.6),
              new feed()
            )
          )
        )
      )
    );

    NamedCommands.registerCommand(
      "Shoot Close",
      new ParallelCommandGroup(
        new WaitCommand(1),
        new SequentialCommandGroup(
          new InstantCommand(() -> arm.setArmState(ARM_STATE.AIM), arm),
          new ParallelCommandGroup(
            new RepeatCommand(
              new InstantCommand(() -> rollers.setShooterVelocity(60))
            ),
            new SequentialCommandGroup(
              new WaitCommand(0.6),
              new feed()
            )
          )
        )
      )
    );

    NamedCommands.registerCommand(
      "SpinUp80",
      new InstantCommand(
        () -> {
          rollers.setShooterVelocity(80);
        }, rollers
      )
    );

    //driver = new CommandXboxController(Constants.DEVICE_IDs.GAMEPAD_DRIVER);
    operator = new CommandXboxController(Constants.DEVICE_IDs.GAMEPAD_OPERATOR);

    configureBindings();

    verbosityChooser.setDefaultOption("LOW", VERBOSITY_LEVEL.LOW);
    verbosityChooser.addOption("HIGH", VERBOSITY_LEVEL.HIGH);
    verbosityChooser.addOption("COMP", VERBOSITY_LEVEL.COMP);
    
    AutoBuilder.buildAutoChooser();

    autoChooser.setDefaultOption("Depression Open", "Depression Open");
    autoChooser.addOption("Depression Tight", "Depression Tight");
    autoChooser.addOption("4 Piece Inside", "4 Piece Inside");
    autoChooser.addOption("4 Piece Inside Conservative", "4 Piece Inside Conservative");
    autoChooser.addOption("Simple Centreline Open", "Simple Centreline Open");
    autoChooser.addOption("Simple Centreline Tight", "Simple Centreline Tight");

    SmartDashboard.putData("VERBOSITY", verbosityChooser);
    SmartDashboard.putData("AUTO", autoChooser);

    swerve.setDefaultCommand(
      new TeleopDrive()
    );

    
    arm.setDefaultCommand(
      new ConditionalCommand(
        new InstantCommand(() -> {
          arm.setArmState(ARM_STATE.LOW);
          led.setLED(LED_STATE.DEFAULT);
        }, arm), 
        new RepeatCommand(
          new InstantCommand(() -> {
            arm.setArmVelocity(0);
            led.setLED(LED_STATE.DEFAULT);
          }, 
          arm)
        ),
        () -> arm.getArmState() != ARM_STATE.MANUAL
      )
    );

    rollers.setDefaultCommand(
      new InstantCommand(() -> rollers.stop(), rollers)
    );
  }

  private void configureBindings() {
    //operator.leftTrigger().whileTrue(new Intake());

    operator
      .axisGreaterThan(1, 0.1)
      .or(operator.axisLessThan(1, -0.1))
        .whileTrue(
          new manualArm()
    );

    
    operator
      .leftTrigger()
        .whileTrue(
            new RepeatCommand(
              new InstantCommand(
              () -> {
                arm.setArmState(ARM_STATE.INTAKE);
                rollers.setIntakeState(INTAKE_STATE.INTAKING);
                rollers.setShooterVelocity(0);
                led.setLED(LED_STATE.INTAKING);
              }, 
              arm, rollers
              )
            ).onlyWhile(
              () -> !rollers.isWithGamePiece()
            ).andThen(
              new InstantCommand(
                () -> {
                  if (rollers.isWithGamePiece()) {
                    CommandScheduler.getInstance().schedule(
                    new RumbleCommand(
                      0.75, 0.25, gamepadConfigs.getInstance().getDriverController(), gamepadConfigs.getInstance().getOperatorController())
                    );
                    led.setLED(LED_STATE.CONFIRM);
                  }
                }
              )
            )
    );
    
    /* 
    operator
      .leftTrigger()
        .whileTrue(
            new RepeatCommand(
              new InstantCommand(
                () -> {
                  arm.setArmState(ARM_STATE.INTAKE);
                  rollers.setIntakeState(INTAKE_STATE.INTAKING);
                }, 
                arm, rollers
              )
            )
    );
    */

    operator
      .rightTrigger()
        .whileTrue(
            new RepeatCommand(
              new InstantCommand(
                () -> {
                  rollers.setIntakeState(INTAKE_STATE.OUTAKE);
                  led.setLED(LED_STATE.INTAKING);
                }, 
                rollers
              )
          )
    );

    operator
      .leftTrigger().and(operator.rightTrigger())
        .whileTrue(
            new RepeatCommand(
              new InstantCommand(
              () -> {
                arm.setArmState(ARM_STATE.INTAKE);
                rollers.setIntakeState(INTAKE_STATE.IDLE);
                led.setLED(LED_STATE.INTAKING);
              }, 
              arm, rollers
            )
          )
    );

    operator
      .leftBumper()
        .whileTrue(
          new RepeatCommand(
            new InstantCommand(
              () -> {
                arm.setArmState(ARM_STATE.HIGH);
                led.setLED(LED_STATE.DEFAULT);
              }, 
              arm
            )
          )
    );

    operator
      .y()
        .whileTrue(
            new RepeatCommand(
              new InstantCommand(
                () -> {
                  arm.setArmState(ARM_STATE.AMP);
                  rollers.setShooterVelocity(30);
                  rollers.setIntakeState(INTAKE_STATE.IDLE);
                  led.setLED(LED_STATE.ALIGNING);
                }, 
                arm, rollers
            )
          )
    );

    operator
      .y().and(operator.rightBumper())
        .whileTrue(
          new RepeatCommand(
              new InstantCommand(
                () -> {
                  arm.setArmState(ARM_STATE.AMP);
                  rollers.setShooterVelocity(30);
                  rollers.setIntakeState(INTAKE_STATE.FEEDING);
                  led.setLED(LED_STATE.SHOOTING);
                }, 
                arm, rollers
            )
          )
    );

    operator
      .b()
        .whileTrue(
            new RepeatCommand(
              new InstantCommand(
                () -> {
                  arm.setArmState(ARM_STATE.LOW);
                  rollers.setShooterVelocity(40);
                  rollers.setIntakeState(INTAKE_STATE.IDLE);
                  led.setLED(LED_STATE.ALIGNING);
                }, 
                arm, rollers
            )
          )
    );

    operator
      .b().and(operator.rightBumper())
        .whileTrue(
          new RepeatCommand(
              new InstantCommand(
                () -> {
                  arm.setArmState(ARM_STATE.LOW);
                  rollers.setShooterVelocity(40);
                  rollers.setIntakeState(INTAKE_STATE.FEEDING);
                  led.setLED(LED_STATE.SHOOTING);
                }, 
                arm, rollers
            )
          )
    );

    operator
      .x()
        .whileTrue(
            new RepeatCommand(
              new InstantCommand(
                () -> {
                  arm.setArmState(ARM_STATE.AIM);
                  rollers.setShooterVelocity(40);
                  rollers.setIntakeState(INTAKE_STATE.IDLE);
                  led.setLED(LED_STATE.ALIGNING);
                }, 
                arm, rollers
            )
          )
    );

    operator
      .x().and(operator.rightBumper())
        .whileTrue(
          new RepeatCommand(
              new InstantCommand(
                () -> {
                  arm.setArmState(ARM_STATE.AIM);
                  rollers.setShooterVelocity(40);
                  rollers.setIntakeState(INTAKE_STATE.FEEDING);
                  led.setLED(LED_STATE.SHOOTING);
                }, 
                arm, rollers
              )
            )
    );

  }

  public Command getAutonomousCommand() { 
    Pose2d pose = PathPlannerAuto.getStaringPoseFromAutoFile(autoChooser.getSelected());

    return new SequentialCommandGroup(
      new InstantCommand(() -> swerve.resetOdom(pose), swerve),
      AutoBuilder.buildAuto(autoChooser.getSelected())
    );
      
      //return autoChooser.getSelected();
  }
}