// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LightsSubsystem extends SubsystemBase {
  private Spark m_Blinkin;
  /** Creates a new LightsSubsystem. */
  public LightsSubsystem() {
    try {
      m_Blinkin  = new Spark(9);
    } catch (Exception e) {
      // TODO: handle exception
      System.out.println("no spark");
    }
  
  }

  public void turnLightsColor(){
    m_Blinkin.set(-.05);
  }

  public void turnLightsOff(){
    m_Blinkin.set(0.99);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
