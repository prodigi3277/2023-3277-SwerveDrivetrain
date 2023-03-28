// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmControlCommand extends CommandBase {
  ArmSubsystem m_arm;
  DoubleSupplier m_extendArmSpeed;
  DoubleSupplier m_pivotArmSpeed;
  DoubleSupplier m_rotateArmSpeed;
  DoubleSupplier m_clawArmSpeed;

  /** Creates a new ArmControlCommand. */
  public ArmControlCommand(ArmSubsystem arm, DoubleSupplier extendArmSpeed,DoubleSupplier pivotArmSpeed,
  DoubleSupplier rotateArmSpeed,DoubleSupplier clawArmSpeed) {
    m_arm = arm;
    m_extendArmSpeed = extendArmSpeed;
    m_pivotArmSpeed = pivotArmSpeed;
    m_rotateArmSpeed = rotateArmSpeed;
    m_clawArmSpeed = clawArmSpeed;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double actualExtendArmSpeed = m_extendArmSpeed.getAsDouble();
    double actualPivotArmSpeed = m_pivotArmSpeed.getAsDouble();
    double actualRotateArmSpeed = m_rotateArmSpeed.getAsDouble();
    double actualClawArmSpeed = m_clawArmSpeed.getAsDouble();
if (Math.abs(actualRotateArmSpeed) <= Math.abs(0.1)   ) {
  m_arm.armRotate(0);
}
else if(Math.abs(actualRotateArmSpeed) > Math.abs(0.1)){
  m_arm.armRotate(actualRotateArmSpeed);

}

if (Math.abs(actualClawArmSpeed) <= Math.abs(0.5)   ) {
  m_arm.claw(0);
}
else if(Math.abs(actualClawArmSpeed) > Math.abs(0.5)){
  m_arm.claw(actualClawArmSpeed);

}

    m_arm.ExtendArm(actualExtendArmSpeed);
    m_arm.armPivot(actualPivotArmSpeed);
    m_arm.claw(actualClawArmSpeed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
