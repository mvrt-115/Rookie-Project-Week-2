// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class PID extends CommandBase {
  // create a field for the  Drivetrain subsystem
  /** Create the field variables for PID */


  /** Creates a new PID. */
  public PID(Drivetrain m, double targetMeters) {
     // Use addRequirements() here to declare subsystem dependencies.
    // initialize field variables
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /** Run the motor at 0.2 speed by using PID */
    // Calculate the error, time, derivative, and add to the integral

    // Calculate the output
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // stop the motor 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
