// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.JoystickSim;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.Lift;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.DriveSpark;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.SparkmaxPID;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (includin)
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final SparkmaxPID sparkmaxpid = new SparkmaxPID();
  private final Climber climber = new Climber();
  private final Joystick joystick1 = new Joystick(OIConstants.kJoystick1);
  private final Joystick joystick2 = new Joystick(OIConstants.kJoystick2);
  
  
  //Autonomous Command Calls
  // private final Command m_PIDRun = new DriveSpark(SmartDashboard.getNumber("Target Position",0),sparkmaxpid,SmartDashboard.getNumber("Position Stop Tolerance",0.001));
  
  //chooser that will determine which command is deployed in each mode
  SendableChooser<Command> autoChooser = new SendableChooser<>();

  //StartEndCommand to run PID to a partical location given PID parameters from SmartDashboard
  // private final Command m_Run4Fun = 
  // new StartEndCommand(
  //   ()-> drivetrain.PIDUpdate(SmartDashboard.getNumber("P Value",.05),SmartDashboard.getNumber("I Value",0.05),SmartDashboard.getNumber("D Value",0.05)),
  //   ()-> drivetrain.PIDSetPointAbsolute(SmartDashboard.getNumber("Position",0.05)),drivetrain
  // );


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    SmartDashboard.putNumber("P Value",0.0);
    SmartDashboard.putNumber("I Value",0.0);
    SmartDashboard.putNumber("D Value",0.0);
    SmartDashboard.putNumber("Target Position",100.0);
    SmartDashboard.putNumber("Target Velocity",10.0);
    SmartDashboard.putNumber("Position Stop Tolerance",0.0);
    SmartDashboard.putBoolean("use kVelocity",true);

    //add options to the Autonomous Chooser
    // autoChooser.setDefaultOption("Move Forward Auto [Using Commands]", m_PIDRun);
    // autoChooser.addOption("Move Forward Auto [Using StartEndCommand]",m_Run4Fun);

    //set climber to control Lift with Joystick1's Y
    // climber.setDefaultCommand(
    //   new RunCommand(
    //     ()-> climber.liftDrive(Lift.kLiftSpeed * joystick1.getY()), climber)
    // );
    
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    //Run PID while button 2 on joystick is being held
    new JoystickButton(joystick1,2).whileHeld(
      new StartEndCommand(
        ()-> sparkmaxpid.PIDSetPoint(SmartDashboard.getNumber("Target Position",0),SmartDashboard.getNumber("P Value",0),SmartDashboard.getNumber("I Value",0),SmartDashboard.getNumber("D Value",0)), 
        ()-> sparkmaxpid.stop(), sparkmaxpid
      )
    );

    //Run Climb PID test code while button 1 on joystick is being held
    new JoystickButton(joystick1, 1).whileHeld(
      new StartEndCommand(
        ()-> climber.PIDSetVelocity(SmartDashboard.getNumber("Target Velocity", 10.0) * joystick1.getY(),SmartDashboard.getNumber("P Value",0),SmartDashboard.getNumber("I Value",0),SmartDashboard.getNumber("D Value",0),SmartDashboard.getBoolean("use kVelocity", true)),
        ()-> climber.stop(), climber
      )
    );
  }

  public Command getAutonomousCommand() {
    // Put the chooser on the dashboard
    SmartDashboard.putData(autoChooser);

    return autoChooser.getSelected();
  }

}
