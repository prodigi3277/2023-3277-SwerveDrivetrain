// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
  WPI_TalonFX extendArm;
  WPI_TalonFX pivotArm;
  WPI_TalonFX rotateArm;
  WPI_TalonSRX claw;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    try {
      extendArm = new WPI_TalonFX(13); //normal 13
    } catch (Exception e) {
      // TODO: handle exception
      System.out.println("extend arm motor not here");
    } 
    try {
      pivotArm = new WPI_TalonFX(14);
    } catch (Exception e) {
      // TODO: handle exception
      System.out.println("pivot arm motor not here");
    } try {
      rotateArm = new WPI_TalonFX(15);
    } catch (Exception e) {
      // TODO: handle exception
      System.out.println("rotate arm motor not here");
    } try {
      claw = new WPI_TalonSRX(16);
    } catch (Exception e) {
      // TODO: handle exception
      System.out.println("claw arm motor not here");
    }
 pivotArm.setNeutralMode(NeutralMode.Brake);

  }

  public void ExtendArm(double extendArmSpeed){
    extendArm.set(extendArmSpeed);
  }

  public void armPivot(double pivotArmSpeed){
    pivotArm.set(pivotArmSpeed);

  }

  public void armRotate(double rotateArmSpeed){
    rotateArm.set(rotateArmSpeed);
  }

  public void claw(double clawArmSpeed){
claw.set(clawArmSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
