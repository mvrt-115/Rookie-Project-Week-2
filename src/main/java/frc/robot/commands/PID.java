// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class PID extends CommandBase {
  // create a field for the  Drivetrain subsystem
  private double kP = 0.05;
  private double kI = 0.0;
  private double kD = 0.00;
  private double lastTime;
  private double lastError;
  private double target;
  private double integral;
  private Drivetrain subsystem;
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
    /** Run the motor at 0.2 speed */
    double error = target - subsystem.getPosition();
    double time = Timer.getFPGATimestamp();
    double derivative = (error-lastError)/(time-lastTime);
    integral += error*(time-lastTime);

    double output = kP*error + kD*derivative + kI*integral;
    output = Math.max(output, -1);
    output = Math.min(output, 1);
    lastTime = time;
    lastError = error;
    subsystem.setSpeeds(output);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // stop the motor 
    subsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
