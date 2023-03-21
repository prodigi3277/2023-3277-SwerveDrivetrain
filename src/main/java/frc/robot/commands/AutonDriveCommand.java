// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutonDriveCommand extends CommandBase {
 private DrivetrainSubsystem m_drivetrain;
 private Timer m_time;
  /** Creates a new ToggleDriveSpeed. */
  public AutonDriveCommand(DrivetrainSubsystem drivetrain) {
    m_drivetrain = drivetrain;
    m_time = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_time.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_time.get() <= 5){
      m_drivetrain.autonDrive();
    }
    if(m_time.get() > 5){
      m_drivetrain.autonStop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_time.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
