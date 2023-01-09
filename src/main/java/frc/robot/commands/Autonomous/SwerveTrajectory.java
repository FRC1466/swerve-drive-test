package frc.robot.commands.Autonomous;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;


public class SwerveTrajectory {
    private DriveSubsystem m_drive;


    private TrajectoryConfig config =
        new TrajectoryConfig(
                AutoConstants.MAX_SPEED_MPS,
                AutoConstants.MAX_ACCELERATION_MPS)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.KINEMATICS);

    // An example trajectory to follow.  All units in meters.
    private Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            config);

    private ProfiledPIDController thetaController =
        new ProfiledPIDController(
            AutoConstants.THETA_CONTROLLER.P,
            AutoConstants.THETA_CONTROLLER.I, 
            AutoConstants.THETA_CONTROLLER.D, 
            AutoConstants.THETA_CONTROLLER_CONSTRAINTS);


    
    public SwerveTrajectory(DriveSubsystem drive) {
        m_drive = drive;
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        drive.resetOdometry(exampleTrajectory.getInitialPose());

    }

    SwerveControllerCommand m_swerveControllerCommand =
            new SwerveControllerCommand(
                exampleTrajectory,
                m_drive::getPose, // Functional interface to feed supplier
                DriveConstants.KINEMATICS,

                // Position controllers
                new PIDController(
                    AutoConstants.X_CONTROLLER.P,
                    AutoConstants.X_CONTROLLER.I, 
                    AutoConstants.X_CONTROLLER.D),
                new PIDController(
                    AutoConstants.Y_CONTROLLER.P, 
                    AutoConstants.X_CONTROLLER.I, 
                    AutoConstants.X_CONTROLLER.D),
                thetaController,
                m_drive::driveFromModuleStates,
                m_drive);
    

    public SwerveControllerCommand getCommand() {
        return m_swerveControllerCommand;
    }
   
    

}
