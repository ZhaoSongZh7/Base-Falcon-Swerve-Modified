// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.feedBall;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Turret;
import frc.robot.commands.feedBall;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class moveAuto extends SequentialCommandGroup {
  Turret m_Turret = new Turret();
  /** Creates a new moveAuto. */
  public moveAuto(Swerve s_Swerve) {
    addCommands(  
      new InstantCommand(() -> {
        s_Swerve.zeroGyro();
      }, s_Swerve));
      s_Swerve.followTrajectoryCommand(PathPlanner.loadPath("test", 2, 3), true)
      .alongWith(new RunCommand(() -> {m_Turret.setOutputSpeed(0.4);}, s_Swerve));
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
  }
}
