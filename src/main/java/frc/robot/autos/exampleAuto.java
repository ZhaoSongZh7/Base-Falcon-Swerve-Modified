package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class exampleAuto extends SequentialCommandGroup {
    // Passing in the swerve drive train as the parameters for the auto constructor.
    public exampleAuto(Swerve s_Swerve){
        // This will load the file "FullAuto.path" and generate it with a max velocity of 4 m/s and a max acceleration of 3 m/s^2
// for every path in the group
// final List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("FullAuto", new PathConstraints(4, 3));

// // This is just an example event map. It would be better to have a constant, global event map
// // in your code that will be used by all path following commands.
// final HashMap<String, Command> eventMap = new HashMap<>();
// eventMap.put("marker1", new PrintCommand("Passed marker 1"));
// eventMap.put("intakeDown", new IntakeDown());
    }

// Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.



    //     TrajectoryConfig config =
    //         new TrajectoryConfig(
    //                 Constants.AutoConstants.kMaxSpeedMetersPerSecond,
    //                 Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    //                 // Find wheel positions using kinematics
    //             .setKinematics(Constants.Swerve.swerveKinematics);

    //     // An example trajectory to follow. All units in meters.
    //     Trajectory exampleTrajectory =
    //         TrajectoryGenerator.generateTrajectory(
    //             // Start at the origin facing the +X direction
    //             new Pose2d(0, 0, new Rotation2d(0)),
    //             // Pass through these two interior waypoints, making an 's' curve path
    //             List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
    //             // End 3 meters straight ahead of where we started, facing forward
    //             new Pose2d(3, 0, new Rotation2d(0)),
    //             config);

    //     var thetaController =
    //         new ProfiledPIDController(
    //             Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
    //     thetaController.enableContinuousInput(-Math.PI, Math.PI);

    //     SwerveControllerCommand swerveControllerCommand =
    //         new SwerveControllerCommand(
    //             exampleTrajectory,
    //             s_Swerve::getPose,
    //             Constants.Swerve.swerveKinematics,
    //             new PIDController(Constants.AutoConstants.kPXController, 0, 0),
    //             new PIDController(Constants.AutoConstants.kPYController, 0, 0),
    //             thetaController,
    //             s_Swerve::setModuleStates,
    //             s_Swerve);


    //     addCommands(
    //         new InstantCommand(() -> s_Swerve.resetOdometry(exampleTrajectory.getInitialPose())),
    //         swerveControllerCommand
    //     );
    // }


}