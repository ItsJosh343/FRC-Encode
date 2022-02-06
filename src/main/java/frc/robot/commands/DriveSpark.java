// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Drivetrain;

public class DriveSpark extends CommandBase {
  /** Creates a new DriveSpark. */

  private double leftMasterSetPoint;
  private double setTarget;

  private final Drivetrain drivetrain;

  public DriveSpark(double target,Drivetrain subDrivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    // addRequirements();
    setTarget = target;
    drivetrain = subDrivetrain;
    addRequirements(subDrivetrain);
  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    leftMasterSetPoint = drivetrain.getLeftMasterPosition() + setTarget;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.setSetPoint(leftMasterSetPoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // while(this.timeSinceInitialized())
    // {
    //   double test = this.timeSinceInitialized()
    // }
    return false;
  }
}
