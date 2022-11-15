// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PIDConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class DriveSubsystem extends SubsystemBase {

  // motor array in a list for easy access (could do dict in future?)
  WPI_TalonFX[][] motors = new WPI_TalonFX[][] {
    new WPI_TalonFX[] {new WPI_TalonFX(DriveConstants.FRONTRIGHT_PORT_DRIVE), new WPI_TalonFX(DriveConstants.FRONTRIGHT_PORT_ROTATE)},
    new WPI_TalonFX[] {new WPI_TalonFX(DriveConstants.FRONTLEFT_PORT_DRIVE), new WPI_TalonFX(DriveConstants.FRONTLEFT_PORT_ROTATE)},
    new WPI_TalonFX[] {new WPI_TalonFX(DriveConstants.BACKRIGHT_PORT_DRIVE), new WPI_TalonFX(DriveConstants.BACKRIGHT_PORT_ROTATE)},
    new WPI_TalonFX[] {new WPI_TalonFX(DriveConstants.BACKLEFT_PORT_DRIVE), new WPI_TalonFX(DriveConstants.BACKLEFT_PORT_ROTATE)}
  }; 

  Translation2d m_frontLeftLocation = new Translation2d(DriveConstants.TRACKWIDTH_METERS/2, DriveConstants.TRACKWIDTH_METERS/2);
  Translation2d m_frontRightLocation = new Translation2d(DriveConstants.TRACKWIDTH_METERS/2, -DriveConstants.TRACKWIDTH_METERS/2);
  Translation2d m_backLeftLocation = new Translation2d(-DriveConstants.TRACKWIDTH_METERS/2, DriveConstants.TRACKWIDTH_METERS/2);
  Translation2d m_backRightLocation = new Translation2d(-DriveConstants.TRACKWIDTH_METERS/2, -DriveConstants.TRACKWIDTH_METERS/2);

  SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
    m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);
  
  ChassisSpeeds speeds = new ChassisSpeeds(0, 0, 0);
  SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(speeds);

  SwerveModuleState frontLeftOptimized = SwerveModuleState.optimize(moduleStates[0], new Rotation2d(0));
  SwerveModuleState frontRightOptimized = SwerveModuleState.optimize(moduleStates[1], new Rotation2d(0));
  SwerveModuleState backLeftOptimized = SwerveModuleState.optimize(moduleStates[2], new Rotation2d(0));
  SwerveModuleState backRightOptimized = SwerveModuleState.optimize(moduleStates[3], new Rotation2d(0));

  SwerveModuleState[] moduleStatesOptimized = new SwerveModuleState[] {frontLeftOptimized, frontRightOptimized, backLeftOptimized, backRightOptimized};
  
  /**
   * Initialize and set configs of the motors
   */
  private void initializeMotors() { //set configs of motors
    for (int i=0; i<motors.length; i++) {
      motors[i][1].configFactoryDefault();
      motors[i][0].configFactoryDefault();
      motors[i][1].set(ControlMode.PercentOutput, 0);
      motors[i][0].set(ControlMode.PercentOutput, 0);
      motors[i][1].setNeutralMode(NeutralMode.Brake);
      motors[i][0].setNeutralMode(NeutralMode.Brake);
      motors[i][1].configNeutralDeadband(0.001);
      motors[i][0].configNeutralDeadband(0.001);
    }
    motors[1][0].setInverted(TalonFXInvertType.Clockwise);
    motors[2][1].setInverted(TalonFXInvertType.Clockwise);
    motors[0][1].setInverted(TalonFXInvertType.Clockwise);
    motors[1][1].setInverted(TalonFXInvertType.Clockwise);
    motors[3][1].setInverted(TalonFXInvertType.Clockwise);

  }
  
  /** Initialize motor PID */
  private void initializePID() {
    for (int i=0; i<motors.length; i++) 
    {
      motors[i][0].configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
      PIDConstants.PID_LOOP_IDX, 
      PIDConstants.TIMEOUT_MS);
      motors[i][1].configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
      PIDConstants.PID_LOOP_IDX, 
      PIDConstants.TIMEOUT_MS);


      /* Config the peak and nominal outputs */
      motors[i][0].configNominalOutputForward(0, PIDConstants.TIMEOUT_MS);
      motors[i][0].configNominalOutputReverse(0, PIDConstants.TIMEOUT_MS);
      motors[i][0].configPeakOutputForward(PIDConstants.DRIVE_GAINS_VELOCITY.PEAK_OUTPUT, PIDConstants.TIMEOUT_MS);
      motors[i][0].configPeakOutputReverse(-PIDConstants.DRIVE_GAINS_VELOCITY.PEAK_OUTPUT, PIDConstants.TIMEOUT_MS);

      motors[i][1].configNominalOutputForward(0, PIDConstants.TIMEOUT_MS);
      motors[i][1].configNominalOutputReverse(0, PIDConstants.TIMEOUT_MS);
      motors[i][1].configPeakOutputForward(PIDConstants.DRIVE_GAINS_VELOCITY.PEAK_OUTPUT, PIDConstants.TIMEOUT_MS);
      motors[i][1].configPeakOutputReverse(-PIDConstants.DRIVE_GAINS_VELOCITY.PEAK_OUTPUT, PIDConstants.TIMEOUT_MS);

      /* Config the Velocity closed loop gains in slot0 */
      motors[i][0].config_kF(PIDConstants.PID_LOOP_IDX, PIDConstants.DRIVE_GAINS_VELOCITY.F, PIDConstants.TIMEOUT_MS);
      motors[i][0].config_kP(PIDConstants.PID_LOOP_IDX, PIDConstants.DRIVE_GAINS_VELOCITY.P, PIDConstants.TIMEOUT_MS);
      motors[i][0].config_kI(PIDConstants.PID_LOOP_IDX, PIDConstants.DRIVE_GAINS_VELOCITY.I, PIDConstants.TIMEOUT_MS);
      motors[i][0].config_kD(PIDConstants.PID_LOOP_IDX, PIDConstants.DRIVE_GAINS_VELOCITY.D, PIDConstants.TIMEOUT_MS);

      motors[i][1].config_kF(PIDConstants.PID_LOOP_IDX, PIDConstants.DRIVE_GAINS_POSITION.F, PIDConstants.TIMEOUT_MS);
      motors[i][1].config_kP(PIDConstants.PID_LOOP_IDX, PIDConstants.DRIVE_GAINS_POSITION.P, PIDConstants.TIMEOUT_MS);
      motors[i][1].config_kI(PIDConstants.PID_LOOP_IDX, PIDConstants.DRIVE_GAINS_POSITION.I, PIDConstants.TIMEOUT_MS);
      motors[i][1].config_kD(PIDConstants.PID_LOOP_IDX, PIDConstants.DRIVE_GAINS_POSITION.D, PIDConstants.TIMEOUT_MS);

      /* Hopefully make not continous */
      motors[i][1].configFeedbackNotContinuous(true, PIDConstants.TIMEOUT_MS);
    }
  }

  


  /**
   * changes motors from optimized SwerveModuleState
   * @param states Optimized SwerveModuleState calculated from other values
   * @param motor motor index
   */
  public void driveFromOptimizedState(int motor) {

    double unitsVel = moduleStates[motor].speedMetersPerSecond / Constants.ConversionConstants.CTRE_NATIVE_TO_MPS;
    motors[motor][0].set(TalonFXControlMode.Velocity, unitsVel);

    double setpoint =  moduleStates[motor].angle.getRadians() / Constants.ConversionConstants.CTRE_TICKS_PER_REV;
    motors[motor][1].set(TalonFXControlMode.Position, setpoint);
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
   * update both normal module states and optimized states
   */
  public void updateModuleStates() {
    moduleStates = m_kinematics.toSwerveModuleStates(speeds);

    frontLeftOptimized = SwerveModuleState.optimize(moduleStates[0], new Rotation2d(motors[1][0].getSelectedSensorPosition()/Constants.ConversionConstants.CTRE_TICKS_PER_REV));
    frontRightOptimized = SwerveModuleState.optimize(moduleStates[1], new Rotation2d(motors[0][0].getSelectedSensorPosition()/Constants.ConversionConstants.CTRE_TICKS_PER_REV));
    backLeftOptimized = SwerveModuleState.optimize(moduleStates[2], new Rotation2d(motors[3][0].getSelectedSensorPosition()/Constants.ConversionConstants.CTRE_TICKS_PER_REV));
    backRightOptimized = SwerveModuleState.optimize(moduleStates[3], new Rotation2d(motors[2][0].getSelectedSensorPosition()/Constants.ConversionConstants.CTRE_TICKS_PER_REV));

    moduleStatesOptimized[0] = frontLeftOptimized;
    moduleStatesOptimized[1] = frontRightOptimized;
    moduleStatesOptimized[2] = backLeftOptimized;
    moduleStatesOptimized[3] = backRightOptimized;
  }

  /**
   * @return length of module states list
   */
  public int getStatesLength() {
    return moduleStatesOptimized.length;
  }

  /**
   * get all motor errors
   * @return list of all errorstates of the motors
   */
  public double[][] getErrorStates() {
    double[][] errorStates = {
      {motors[0][1].getClosedLoopError(),  motors[0][0].getClosedLoopError()},
      {motors[1][1].getClosedLoopError(),  motors[1][0].getClosedLoopError()},
      {motors[2][1].getClosedLoopError(),  motors[2][0].getClosedLoopError()},
      {motors[3][1].getClosedLoopError(),  motors[3][0].getClosedLoopError()}
    };
    return errorStates;
  }




  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    initializeMotors();
    initializePID();
  }

  @Override
  public void periodic() {

  }


  /**
   * Sets the maximum output of the PID in motors
   * @param peakOutput the peak output
   */
  public void setPeakOutputPID(double peakOutput) {
    for (int i = 0; i < motors.length; i++) {
      motors[i][0].configPeakOutputForward(peakOutput);
      motors[i][1].configPeakOutputForward(peakOutput);
      motors[i][0].configPeakOutputReverse(-peakOutput);
      motors[i][1].configPeakOutputReverse(-peakOutput);
    }
  }

}
