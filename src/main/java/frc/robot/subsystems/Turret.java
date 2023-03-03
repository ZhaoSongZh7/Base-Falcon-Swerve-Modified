// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {
  public Solenoid turretPiston;
  public Compressor comp;

  /** Creates a new Turret. */
  public Turret() {
    turretPiston = new Solenoid(PneumaticsModuleType.REVPH, 1);
    comp = new Compressor(PneumaticsModuleType.REVPH);
    comp.enableAnalog(0.6,0.7);
  }

  public void togglePiston(boolean extend) {
    turretPiston.set(extend);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
