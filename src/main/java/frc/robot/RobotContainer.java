// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.RobotMode;
import frc.robot.commands.Drive.TeleopSwerve;
import frc.robot.subsystems.Drive.Swerve;
import frc.robot.subsystems.Lights.Lights;
import frc.robot.commands.Lights.SolidColor;


public class RobotContainer {

  private RobotMode robotMode = RobotMode.DISABLED;
  /* Controllers */
  private final CommandXboxController driver = new CommandXboxController(Constants.Operators.driver);
  private SendableChooser<Command> m_chooser = new SendableChooser<>();

  /* Subsystems */
  final Swerve s_Swerve = new Swerve();
 final Lights lights = new Lights();

  public RobotContainer() {
    configureButtonBindings();

  }

  public void setRobotMode(RobotMode mode) {
    robotMode = mode;
  }

  public void disabledActions() {

  }

  public void teleopInit() {
    lights.clearBuffer();
   

  }

  public void disabledInit() {
    new SolidColor(lights, Constants.LightsConstants.Colors.GREEN);
     
  }

  private void configureButtonBindings() {
    // default commands
    /* Driver Controller */
    s_Swerve.setDefaultCommand(new TeleopSwerve(
        s_Swerve,
        () -> -driver.getLeftY(),
        () -> -driver.getLeftX(),
        () -> -driver.getRightX(),
        () -> driver.getRightTriggerAxis()));

    driver.back().onTrue(new InstantCommand(s_Swerve::zeroGyro));

  }

  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}