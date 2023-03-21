// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LightsPWM extends SubsystemBase {
  AddressableLED m_ledStrip;
  /** Creates a new LightsPWM. */
  public LightsPWM() {
    try {
    m_ledStrip = new AddressableLED(9);
      
    } catch (Exception e) {
      // TODO: handle exception
    }

    m_ledStrip.setLength(45);
  }

  private void setRGB(int red, int green, int blue){
    //for

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
