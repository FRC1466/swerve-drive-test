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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PIDConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;

public class DriveSubsystem extends SubsystemBase {

  private final WPI_TalonSRX gyro_motor = new WPI_TalonSRX(DriveConstants.GYRO_PORT);
  private final WPI_PigeonIMU gyro = new WPI_PigeonIMU(gyro_motor);
  private final SwerveModule frontLeftModule = new SwerveModule(
    DriveConstants.FRONTLEFT_PORT_DRIVE, 
    DriveConstants.FRONTLEFT_PORT_ROTATE,
    DriveConstants.FRONTLEFT_PORT_CANCODER,
    DriveConstants.FRONTLEFT_OFFSET,
    true,
    TalonFXInvertType.CounterClockwise);
  private final SwerveModule frontRightModule = new SwerveModule(
    DriveConstants.FRONTRIGHT_PORT_DRIVE, 
    DriveConstants.FRONTRIGHT_PORT_ROTATE,
    DriveConstants.FRONTRIGHT_PORT_CANCODER,
    DriveConstants.FRONTRIGHT_OFFSET,
    true,
    TalonFXInvertType.CounterClockwise);
  private final SwerveModule backLeftModule = new SwerveModule(
    DriveConstants.BACKLEFT_PORT_DRIVE, 
    DriveConstants.BACKLEFT_PORT_ROTATE,
    DriveConstants.BACKLEFT_PORT_CANCODER,
    DriveConstants.BACKLEFT_OFFSET,
    true,
    TalonFXInvertType.CounterClockwise);
  private final SwerveModule backRightModule = new SwerveModule(
    DriveConstants.BACKRIGHT_PORT_DRIVE, 
    DriveConstants.BACKRIGHT_PORT_ROTATE,
    DriveConstants.BACKRIGHT_PORT_CANCODER,
    DriveConstants.BACKRIGHT_OFFSET,
    true,
    TalonFXInvertType.CounterClockwise);


  private ChassisSpeeds speeds = new ChassisSpeeds(0, 0, 0);
  private SwerveModuleState[] moduleStates = DriveConstants.KINEMATICS.toSwerveModuleStates(speeds);

  private final Pose2d initialPose = new Pose2d(5.0, 13.5, new Rotation2d(0));
  private SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(DriveConstants.KINEMATICS, getGyroHeading(), initialPose);

  double[] help = new double[] {};
  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    gyro.reset();
    initializePIDUpdate();
    SmartDashboard.putBoolean("is360BarrierFix", false);
    SmartDashboard.putNumberArray("gyro offsets", 
      new double[] {
        DriveConstants.FRONTLEFT_OFFSET,
        DriveConstants.FRONTRIGHT_OFFSET,
        DriveConstants.BACKLEFT_OFFSET,
        DriveConstants.BACKRIGHT_OFFSET
      });
        SmartDashboard.putNumber("0 cc", 0.0);
        SmartDashboard.putNumber("1 cc", 0.0);
        SmartDashboard.putNumber("2 cc", 0.0);
        SmartDashboard.putNumber("3 cc", 0.0);
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

  public void changeMotorInversion(boolean[] i) {
      frontLeftModule.changeDriveMotorInversion(i[0]);
      frontRightModule.changeDriveMotorInversion(i[1]);
      backLeftModule.changeDriveMotorInversion(i[2]);
      backRightModule.changeDriveMotorInversion(i[3]);
  }

  /**
   * set ChassisSpeeds from a time in the Trajectory
   * @param time time in seconds in the Trajectory
   */
  public void updateSpeedsTrajectory(double time, RamseteController ramsete, Trajectory trajectory) {
    speeds = ramsete.calculate(m_odometry.getPoseMeters(), trajectory.sample(time));
  }

  public void resetAngleByCancoderOffset(double[] list) {
    frontLeftModule.resetAngleEncoder(list[0]);
    frontRightModule.resetAngleEncoder(list[1]);
    backLeftModule.resetAngleEncoder(list[2]);
    backRightModule.resetAngleEncoder(list[3]);
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
    SmartDashboard.putNumber("speedsRad", rad);
    speeds.vxMetersPerSecond = vx;
    SmartDashboard.putNumber("speedsvx", vx);
    speeds.vyMetersPerSecond = vy;
    SmartDashboard.putNumber("speedsvy", vy);
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
  public void updatePIDConfigs() {
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

  public void updatePIDConstants() {
    PIDConstants.DRIVE_GAINS_POSITION.P = SmartDashboard.getNumber("P_pos", 0);
    PIDConstants.DRIVE_GAINS_POSITION.I = SmartDashboard.getNumber("I_pos", 0);
    PIDConstants.DRIVE_GAINS_POSITION.D = SmartDashboard.getNumber("D_pos", 0);
    PIDConstants.DRIVE_GAINS_POSITION.F = SmartDashboard.getNumber("F_pos", 0);
    PIDConstants.DRIVE_GAINS_POSITION.IZONE = SmartDashboard.getNumber("Izone_pos", 0);

    PIDConstants.DRIVE_GAINS_VELOCITY.P = SmartDashboard.getNumber("P_vel", 0);
    PIDConstants.DRIVE_GAINS_VELOCITY.I = SmartDashboard.getNumber("I_vel", 0);
    PIDConstants.DRIVE_GAINS_VELOCITY.D = SmartDashboard.getNumber("D_vel", 0);
    PIDConstants.DRIVE_GAINS_VELOCITY.F = SmartDashboard.getNumber("F_vel", 0);
    PIDConstants.DRIVE_GAINS_VELOCITY.IZONE = SmartDashboard.getNumber("Izone_vel", 0);
  }

  private void initializePIDUpdate() {
    SmartDashboard.putNumber("P_pos", PIDConstants.DRIVE_GAINS_POSITION.P);
    SmartDashboard.putNumber("I_pos", PIDConstants.DRIVE_GAINS_POSITION.I);
    SmartDashboard.putNumber("D_pos", PIDConstants.DRIVE_GAINS_POSITION.D);
    SmartDashboard.putNumber("F_pos", PIDConstants.DRIVE_GAINS_POSITION.F);
    SmartDashboard.putNumber("Izone_pos", PIDConstants.DRIVE_GAINS_POSITION.IZONE);

    SmartDashboard.putNumber("P_vel", PIDConstants.DRIVE_GAINS_VELOCITY.P);
    SmartDashboard.putNumber("I_vel", PIDConstants.DRIVE_GAINS_VELOCITY.I);
    SmartDashboard.putNumber("D_vel", PIDConstants.DRIVE_GAINS_VELOCITY.D);
    SmartDashboard.putNumber("F_vel", PIDConstants.DRIVE_GAINS_VELOCITY.F);
    SmartDashboard.putNumber("Izone_vel", PIDConstants.DRIVE_GAINS_VELOCITY.IZONE);
  } 



  @Override
  public void periodic() {
    updateRobotPose();
  }

}