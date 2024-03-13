// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team9062.robot;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.team9062.robot.Subsystems.Arm;
import frc.team9062.robot.Subsystems.LEDSubsystem;
import frc.team9062.robot.Subsystems.Rollers;
import frc.team9062.robot.Subsystems.Drive.SwerveSubsystem;
import frc.team9062.robot.Subsystems.LEDSubsystem.LED_STATE;
import frc.team9062.robot.Util.lib.LimelightHelpers;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  //private LEDSubsystem led;

  private RobotContainer m_robotContainer;
  private SendableChooser<Boolean> intakeDirection = new SendableChooser<>();

  public Robot() {
    m_robotContainer = new RobotContainer();

    addPeriodic(Arm.getInstance().controls, 0.01, 0.005);
  }

  @Override
  public void robotInit() {
   //led = LEDSubsystem.getInstance();

   //LimelightHelpers.setLEDMode_ForceOff("limelight");

    intakeDirection.setDefaultOption("Forward", true);
    intakeDirection.setDefaultOption("Reverse", false);

    SmartDashboard.putData(intakeDirection);

   for (int port = 5000; port < 5707; port++) {
    PortForwarder.add(port, "limelight.local", port);
   }
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    intakeDirection.onChange(
      (direction) -> {
        Rollers.getInstance().setIntakeDirection(direction);
      }
    );
    
    //LimelightHelpers.setLEDMode_ForceOff("limelight");
  }

  @Override
  public void disabledInit() {
    //led.setLED(LED_STATE.DISABLED);
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {
    //led.setLED(LED_STATE.DEFAULT);
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    //SwerveSubsystem.getInstance().start_timer();
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
   //SwerveSubsystem.getInstance().stop_timer();
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    //SwerveSubsystem.getInstance().start_timer();
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
    //SwerveSubsystem.getInstance().start_timer();
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
