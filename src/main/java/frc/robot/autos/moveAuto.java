// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import java.lang.reflect.Field;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.FeedBall;
import frc.robot.commands.IntakeBall;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Turret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class moveAuto extends SequentialCommandGroup {
  /** Creates a new moveAuto. */
  public moveAuto(Swerve s_Swerve, Turret turret, Intake intake) {
    // addCommands(
    //   new SequentialCommandGroup(s_Swerve.followTrajectoryCommand(PathPlanner.loadPath("test", new PathConstraints(4, 3)), true),
    //   new FeedBall(turret).withTimeout(4)
    // ));

    // addCommands(
    //   new SequentialCommandGroup(s_Swerve.followTrajectoryCommand(PathPlanner.loadPath("New Path", new PathConstraints(4, 3)), true),
    //   new IntakeBall(intake).withTimeout(2)
    // ));

    addCommands(
      new SequentialCommandGroup(s_Swerve.followTrajectoryCommand(PathPlanner.loadPath("One Ball Move", new PathConstraints(4, 3)), true),
      new IntakeBall(intake).withTimeout(2)
    ));

    addCommands(
      new SequentialCommandGroup(s_Swerve.followTrajectoryCommand(PathPlanner.loadPath("One Ball Shoot", new PathConstraints(4, 3)), true),
      new FeedBall(turret).withTimeout(2)
    ));

    // addCommands(  
    //   s_Swerve.followTrajectoryCommand(PathPlanner.loadPath("test", 4, 3), true)
    //   // .alongWith(new FeedBall(turret).alongWith(new WaitCommand(1).andThen(new StopFeeder(turret)))
    //   .alongWith(new FeedBall(turret).withTimeout(4).andThen(new StopFeeder(turret))
    //   )
      // new InstantCommand(() -> {
      //   s_Swerve.zeroGyro();
      // }, s_Swerve));
      // s_Swerve.followTrajectoryCommand(PathPlanner.loadPath("test", 2, 3), true)
      // .alongWith(new RunCommand(() -> {m_Turret.setOutputSpeed(0.4);}, s_Swerve));
    //);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
  }
}
