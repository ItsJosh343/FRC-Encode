// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.DriveSpark;
import frc.robot.subsystems.Drivetrain;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (includin)
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final Drivetrain drivetrain = new Drivetrain();
  private final Joystick joystick1 = new Joystick(OIConstants.kJoystick1);
  private final Joystick joystick2 = new Joystick(OIConstants.kJoystick2);
  
  //Autonomous Command Calls
  private final Command m_PIDRun = new DriveSpark(1,drivetrain);


  private final Command m_Run4Fun = 
  new StartEndCommand(
    ()-> drivetrain.(),
    ()-> drivetrain.setSetPoint(100),drivetrain
  );

  //   new DriveForward();

  // SendableChooser<Command> m_chooser = new SendableChooser<>();

  //Setup multiple autonmous codes to be chosen from SimpleDisplay
  // m_chooser.setDefaultOption("Autonomous Final",m_finalAuto);
  // m_chooser.addOption("Move Forward Auto",m_moveForwardAuto);

  // Put the chooser on the dashboard
  // SmartDashboard.putData(m_chooser);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //drivetrain.setDefaultCommand(new RunCommand(
      //() -> drivetrain.drive.tankDrive(-joystick1.getY(), joystick2.getY()), drivetrain));


    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  public Command getAutonomousCommand() {
    return null;
    // return m_chooser.getSelected();
  }

}
