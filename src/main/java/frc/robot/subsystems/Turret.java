// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.FeedBall;

public class Turret extends SubsystemBase {
  public Solenoid turretPiston;
  public Compressor comp;
  public WPI_TalonFX feederMotor;
  public WPI_TalonFX topTurretMotor;
  public WPI_TalonFX bottomTurretMotor;

  /** Creates a new Turret. */
  public Turret() {
    turretPiston = new Solenoid(PneumaticsModuleType.REVPH, 1);
    feederMotor = new WPI_TalonFX(23);
    topTurretMotor = new WPI_TalonFX(24);
    bottomTurretMotor = new WPI_TalonFX(25);

    comp = new Compressor(PneumaticsModuleType.REVPH);
    comp.enableAnalog(75, 90);

  }

  public void togglePiston(boolean extend) {
    turretPiston.set(extend);
  }

  public void setOutputSpeed(double speed) {
    feederMotor.set(speed * 2);
    topTurretMotor.set(-speed * 2);
    bottomTurretMotor.set(-speed);
  }

  public void stop() {
    feederMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
