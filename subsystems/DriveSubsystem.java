// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;

public class DriveSubsystem extends SubsystemBase {

  private final WPI_PigeonIMU gyro = new WPI_PigeonIMU(DriveConstants.GYRO_PORT);
  private final SwerveModule frontLeftModule = new SwerveModule(DriveConstants.FRONTLEFT_PORT_DRIVE, DriveConstants.FRONTLEFT_PORT_ROTATE);
  private final SwerveModule frontRightModule = new SwerveModule(DriveConstants.FRONTRIGHT_PORT_DRIVE, DriveConstants.FRONTRIGHT_PORT_ROTATE);
  private final SwerveModule backLeftModule = new SwerveModule(DriveConstants.BACKLEFT_PORT_DRIVE, DriveConstants.BACKLEFT_PORT_ROTATE);
  private final SwerveModule backRightModule = new SwerveModule(DriveConstants.BACKRIGHT_PORT_DRIVE, DriveConstants.BACKRIGHT_PORT_ROTATE);


  private ChassisSpeeds speeds = new ChassisSpeeds(0, 0, 0);
  private SwerveModuleState[] moduleStates = DriveConstants.KINEMATICS.toSwerveModuleStates(speeds);

  private final Pose2d initialPose = new Pose2d(5.0, 13.5, new Rotation2d(0));
  private SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(DriveConstants.KINEMATICS, getGyroHeading(), initialPose);

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    gyro.reset();
  }
  
  /**
   * 
   * @return Rotation2d of gyro
   */
  public Rotation2d getGyroHeading() {
    return gyro.getRotation2d();
  }

  /**
   * 
   * @return Pose2d of the robot from odometry
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * reset odometry of the robot from a given pose
   * @param pose Pose2d that the robot is at
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(pose, getGyroHeading());
  }

  /**
   * drive from module states list
   * @param states list of SwerveModuleStates that correspond to the robot
   */
  public void driveFromModuleStates(SwerveModuleState[] states) {
    frontLeftModule.setDesiredState(states[0]);
    frontRightModule.setDesiredState(states[1]);
    backLeftModule.setDesiredState(states[2]);
    backRightModule.setDesiredState(states[3]);
  }

  /**
   * update the odometry of the robot with current pose of the robot
   */
  public void updateRobotPose() {
    m_odometry.update(
      getGyroHeading(),
      frontLeftModule.getState(), 
      frontRightModule.getState(), 
      backLeftModule.getState(), 
      backRightModule.getState()
      );
  }

  /**
   * set ChassisSpeeds from a time in the Trajectory
   * @param time time in seconds in the Trajectory
   */
  public void updateSpeedsTrajectory(double time, RamseteController ramsete, Trajectory trajectory) {
    speeds = ramsete.calculate(m_odometry.getPoseMeters(), trajectory.sample(time));
  }

  /**
   * drive robot from current module states in the class
   */
  public void drive() {
    frontLeftModule.setDesiredState(moduleStates[0]);
    frontRightModule.setDesiredState(moduleStates[1]);
    backLeftModule.setDesiredState(moduleStates[2]);
    backRightModule.setDesiredState(moduleStates[3]);
  }

  /**
   * update speeds kinematics class 
   * @param rad rad/s speed of robot
   * @param vx horizontal velocity in m/s
   * @param vy vertical velocity in m/s
   */
  public void updateSpeeds(double rad, double vx, double vy) {
    speeds.omegaRadiansPerSecond = rad;
    speeds.vxMetersPerSecond = vx;
    speeds.vyMetersPerSecond = vy;
  }

  /**
   * update speeds from field relative setup
   * @param rad rad/s speed of robot
   * @param vx horizontal velocity in m/s
   * @param vy vertical velocity in m/s
   */
  public void updateSpeedsFieldRelative(double rad, double vx, double vy) {
    speeds = ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, rad, getGyroHeading());
  }

  /**
   * update normal moduleStates
   */
  public void updateModuleStates() {
    moduleStates = DriveConstants.KINEMATICS.toSwerveModuleStates(speeds);
  }

  /**
   * update PID in the module substates from constants
   */
  public void updatePID() {
    frontLeftModule.updatePID();
    frontRightModule.updatePID();
    backLeftModule.updatePID();
    backRightModule.updatePID();
  }

  /**
   * @return length of module states list
   */
  public int getStatesLength() {
    return moduleStates.length;
  }

  /**
   * get all motor errors
   * @return list of all errorstates of the motors
   */
  public double[][] getErrorStates() {
    return new double[][] {
      frontLeftModule.getErrorStates(),
      frontRightModule.getErrorStates(),
      backLeftModule.getErrorStates(),
      backRightModule.getErrorStates()
    };
  }

  /**
   * get all motor positions
   * @return list of all positions of the motors
   */
  public double[][] getPositions() {
    return new double[][] {
      frontLeftModule.getPosition(),
      frontRightModule.getPosition(),
      backLeftModule.getPosition(),
      backRightModule.getPosition()
    };
  }

  /**
   * get all motor velocities
   * @return list of all velocities of the motors
   */
  public double[][] getVelocities() {
    return new double[][] {
      frontLeftModule.getVelocity(),
      frontRightModule.getVelocity(),
      backLeftModule.getVelocity(),
      backRightModule.getVelocity()
    };
  }

  @Override
  public void periodic() {
    updateRobotPose();
  }

}
