// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutonDriveAndPlaceCommand extends CommandBase {
  DrivetrainSubsystem m_drive;
  ArmSubsystem m_arm;
  Timer m_time;
  /** Creates a new AutonDriveAndPlaceCommand. */
  public AutonDriveAndPlaceCommand(DrivetrainSubsystem drive, ArmSubsystem arm) {
    m_drive = drive;
    m_arm = arm;
    m_time = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive, m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_time.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_time.get() <0.5){
      //m_drive.autonStop();
      m_arm.ExtendArm(0.75);
    }
    if (m_time.get()>0.5) {
      m_arm.ExtendArm(0);
      m_arm.armRotate(1);
    }
    if (m_time.get()>0.9) {
      m_arm.armRotate(0);
      m_arm.claw(1);
      

    }
    if(m_time.get() > 1.4){
   //   m_arm.ExtendArm(-0.75);

     // m_drive.autonDrive();
    }
    if(m_time.get() > 3){
      m_arm.ExtendArm(0);

     // m_drive.autonStop();
    }
    if(m_time.get() > 4){
  m_drive.autonDrive();
    }
    if(m_time.get() > 10){ //normally 6.475 / 6.775
      m_drive.autonStop();
      m_drive.isLocked = true;
    }
    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.isLocked = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
