// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PIDConstants;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import java.lang.Math;

public class DriveSubsystem extends SubsystemBase {

  // motor array in a list for easy access (could do dict in future?)
  WPI_TalonFX[][] motors = new WPI_TalonFX[][] {
    new WPI_TalonFX[] {new WPI_TalonFX(DriveConstants.FRONTRIGHT_PORT_DRIVE), new WPI_TalonFX(DriveConstants.FRONTRIGHT_PORT_ROTATE)},
    new WPI_TalonFX[] {new WPI_TalonFX(DriveConstants.FRONTLEFT_PORT_DRIVE), new WPI_TalonFX(DriveConstants.FRONTLEFT_PORT_ROTATE)},
    new WPI_TalonFX[] {new WPI_TalonFX(DriveConstants.BACKRIGHT_PORT_DRIVE), new WPI_TalonFX(DriveConstants.BACKRIGHT_PORT_ROTATE)},
    new WPI_TalonFX[] {new WPI_TalonFX(DriveConstants.BACKLEFT_PORT_DRIVE), new WPI_TalonFX(DriveConstants.BACKLEFT_PORT_ROTATE)}
  }; 

  // make the kinematics for the robot
  Translation2d m_frontLeftLocation = new Translation2d(DriveConstants.TRACKWIDTH_METERS/2, DriveConstants.TRACKWIDTH_METERS/2);
  Translation2d m_frontRightLocation = new Translation2d(DriveConstants.TRACKWIDTH_METERS/2, -DriveConstants.TRACKWIDTH_METERS/2);
  Translation2d m_backLeftLocation = new Translation2d(-DriveConstants.TRACKWIDTH_METERS/2, DriveConstants.TRACKWIDTH_METERS/2);
  Translation2d m_backRightLocation = new Translation2d(-DriveConstants.TRACKWIDTH_METERS/2, -DriveConstants.TRACKWIDTH_METERS/2);

  SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
    m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);
  
    // initialize speeds and modulestates, along with optimizations
  ChassisSpeeds speeds = new ChassisSpeeds(0, 0, 0);
  SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(speeds);

  SwerveModuleState frontLeftOptimized = SwerveModuleState.optimize(moduleStates[0], new Rotation2d(0));
  SwerveModuleState frontRightOptimized = SwerveModuleState.optimize(moduleStates[1], new Rotation2d(0));
  SwerveModuleState backLeftOptimized = SwerveModuleState.optimize(moduleStates[2], new Rotation2d(0));
  SwerveModuleState backRightOptimized = SwerveModuleState.optimize(moduleStates[3], new Rotation2d(0));

  SwerveModuleState[] moduleStatesOptimized = new SwerveModuleState[] {frontLeftOptimized, frontRightOptimized, backLeftOptimized, backRightOptimized};

  // The robot's drive
  

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
  }
  

  private void initializePID() { //set configs of PID
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
      motors[i][0].config_kF(PIDConstants.TIMEOUT_MS, PIDConstants.DRIVE_GAINS_VELOCITY.F, PIDConstants.TIMEOUT_MS);
      motors[i][0].config_kP(PIDConstants.TIMEOUT_MS, PIDConstants.DRIVE_GAINS_VELOCITY.P, PIDConstants.TIMEOUT_MS);
      motors[i][0].config_kI(PIDConstants.TIMEOUT_MS, PIDConstants.DRIVE_GAINS_VELOCITY.I, PIDConstants.TIMEOUT_MS);
      motors[i][0].config_kD(PIDConstants.TIMEOUT_MS, PIDConstants.DRIVE_GAINS_VELOCITY.D, PIDConstants.TIMEOUT_MS);

      motors[i][1].config_kF(PIDConstants.PID_LOOP_IDX, PIDConstants.DRIVE_GAINS_POSITION.F, PIDConstants.TIMEOUT_MS);
      motors[i][1].config_kP(PIDConstants.PID_LOOP_IDX, PIDConstants.DRIVE_GAINS_POSITION.P, PIDConstants.TIMEOUT_MS);
      motors[i][1].config_kI(PIDConstants.PID_LOOP_IDX, PIDConstants.DRIVE_GAINS_POSITION.I, PIDConstants.TIMEOUT_MS);
      motors[i][1].config_kD(PIDConstants.PID_LOOP_IDX, PIDConstants.DRIVE_GAINS_POSITION.D, PIDConstants.TIMEOUT_MS);
    }
  }


  /**
   * drives from an optimizes swervemodulestate
   * @param motor the motor grouping that it drives from
   */
  public void driveFromOptimizedState(int motor) {

    double percent = Math.tanh(moduleStatesOptimized[motor].speedMetersPerSecond); //TODO: proper conversion here
    double unitsVel = moduleStates[motor].speedMetersPerSecond / Constants.ConversionConstants.CTRE_NATIVE_TO_MPS;
    // System.out.println("unitsVel: " + unitsVel);
    motors[motor][0].set(TalonFXControlMode.Velocity, unitsVel);

    double setpoint =  moduleStates[motor].angle.getRadians() / (2*Math.PI) * Constants.ConversionConstants.CTRE_TICKS_PER_REV * 1.5;
    // System.out.println("setpoint: " + setpoint);

    motors[motor][1].set(TalonFXControlMode.Position, setpoint);
  }

  /**
   * update the speeds class in the subsystem
   * @param rad the rotation in radians
   * @param vx the horizontal speed in m/s
   * @param vy the vertical speed in m/s
   */
  public void updateSpeeds(double rad, double vx, double vy) {
    speeds.omegaRadiansPerSecond = rad;
    speeds.vxMetersPerSecond = vx;
    System.out.println(speeds.vxMetersPerSecond);
    speeds.vyMetersPerSecond = vy;
    System.out.println(speeds.vyMetersPerSecond);
  }

  /**
   * update the modulestates in the subsystem
   */
  public void updateModuleStates() {
    moduleStates = m_kinematics.toSwerveModuleStates(speeds);

    frontLeftOptimized = SwerveModuleState.optimize(moduleStates[0], new Rotation2d(motors[1][0].getSelectedSensorPosition()/Constants.ConversionConstants.CTRE_TICKS_PER_REV*(2*Math.PI)));
    frontRightOptimized = SwerveModuleState.optimize(moduleStates[1], new Rotation2d(motors[0][0].getSelectedSensorPosition()/Constants.ConversionConstants.CTRE_TICKS_PER_REV*(2*Math.PI)));
    backLeftOptimized = SwerveModuleState.optimize(moduleStates[2], new Rotation2d(motors[3][0].getSelectedSensorPosition()/Constants.ConversionConstants.CTRE_TICKS_PER_REV*(2*Math.PI)));
    backRightOptimized = SwerveModuleState.optimize(moduleStates[3], new Rotation2d(motors[2][0].getSelectedSensorPosition()/Constants.ConversionConstants.CTRE_TICKS_PER_REV*(2*Math.PI)));

    moduleStatesOptimized[0] = frontLeftOptimized;
    moduleStatesOptimized[1] = frontRightOptimized;
    moduleStatesOptimized[2] = backLeftOptimized;
    moduleStatesOptimized[3] = backRightOptimized;
  }

  /**
   * 
   * @return the length of the modulestates
   */
  public int getStatesLength() {
    return moduleStatesOptimized.length;
  }

  /**
   * 
   * @return a list of all motors' error states
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
