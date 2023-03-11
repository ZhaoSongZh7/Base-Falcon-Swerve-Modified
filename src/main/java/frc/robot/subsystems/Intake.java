// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  public WPI_TalonFX intakeMotor;
  public WPI_TalonFX rejectBallMotor;

  /** Creates a new Intake. */
  public Intake() {
    intakeMotor = new WPI_TalonFX(29);
    rejectBallMotor = new WPI_TalonFX(16);
  }

  public void setOutputSpeed(double speed) {
    intakeMotor.set(speed);
    rejectBallMotor.set(-speed);
  }

  public void stop() {
    intakeMotor.set(0);
    rejectBallMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
