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

  Drivetrain drivetrain;
  double target;
  double lastTime;
  double area; 
  double lastError;

  final double kP = 0.1;
  final double kI = 0;
  final double kD = 0;


  /** Creates a new PID. */
  public PID(Drivetrain m, double targetMeters) {
     // Use addRequirements() here to declare subsystem dependencies.
    // initialize field variables

    drivetrain = m;
    target = targetMeters;
    area = 0;
    lastTime = 0;
    lastError = 0;

    addRequirements(m);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lastTime = Timer.getFPGATimestamp();
    lastError = target - drivetrain.getPosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /** Run the motor at 0.2 speed by using PID */
    // Calculate the error, time, derivative, and add to the integral
    double error = target - drivetrain.getPosition();
    double dt = Timer.getFPGATimestamp() - lastTime;
    double de = error - lastError;
    double derivative = de/dt;
    
    area += error * dt;

    // Calculate the output
    double output = kP * error + kI * area + kD * derivative;

    output = Math.min(Math.max(-1, output), 1);

    // Run motor
    drivetrain.setSpeeds(output);

    // update last variables
    lastTime = Timer.getFPGATimestamp();
    lastError = target - drivetrain.getPosition();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // stop the motor 
    drivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(target - drivetrain.getPosition()) < 0.1;
  }
}
