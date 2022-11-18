package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ConversionConstants;
import frc.robot.Constants.PIDConstants;

public class SwerveModule {
    private WPI_TalonFX[] motors;
    private WPI_CANCoder cancoder;
    private int m_cancoderPort;
    
    /**
     * Initialize a Swerve Module
     */
    public SwerveModule(
        int drivePort,
        int rotationPort,
        int cancoderPort
    ) {
        motors = new WPI_TalonFX[] {
            new WPI_TalonFX(drivePort),
            new WPI_TalonFX(rotationPort)
        };
        cancoder = new WPI_CANCoder(cancoderPort);
        m_cancoderPort = cancoderPort;

        initializeMotors();
        initializeMotorsPID();
        initializeCancoder();
        resetAngleEncoder(cancoder.getAbsolutePosition()/360 * ConversionConstants.CTRE_TICKS_PER_REV);
        resetDriveEncoder(0.0);

        if (m_cancoderPort == 10) {
            resetAngleEncoder((cancoder.getAbsolutePosition()+45+180)/360 * ConversionConstants.CTRE_TICKS_PER_REV);
        }
        if (m_cancoderPort == 11) {
            resetAngleEncoder((cancoder.getAbsolutePosition()-45)/360 * ConversionConstants.CTRE_TICKS_PER_REV);
        }
        if (m_cancoderPort == 12) {
            resetAngleEncoder((cancoder.getAbsolutePosition()-45)/360 * ConversionConstants.CTRE_TICKS_PER_REV);
        }
        if (m_cancoderPort == 9) {
            resetAngleEncoder((cancoder.getAbsolutePosition()+45)/360 * ConversionConstants.CTRE_TICKS_PER_REV);
        }
    }

    /**
     * get current state of swerve module
     * @return SwerveModuleState created from current encoder layouts
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
            motors[1].getSelectedSensorVelocity() * ConversionConstants.CTRE_NATIVE_TO_MPS, 
            new Rotation2d(motors[0].getSelectedSensorPosition() * ConversionConstants.CTRE_TICKS_PER_REV % ConversionConstants.CTRE_TICKS_PER_REV)
            );
    }

    /**
     * get list of errors
     * @return list of doubles for both motor errors, 0: drive, 1: position
     */
    public double[] getErrorStates() {
        return new double[] {motors[0].getClosedLoopError(), motors[1].getClosedLoopError()};
    }

    /**
     * get list of positions
     * @return list of doubles for both motor positions, 0: drive, 1: position
     */
    public double[] getPosition() {
        return new double[] {motors[0].getSelectedSensorPosition(), motors[1].getSelectedSensorPosition()};
    }

    public Rotation2d getCancoderAngle() {
        return Rotation2d.fromDegrees(cancoder.getAbsolutePosition());
    }

    /**
     * get list of velocities
     * @return list of doubles for both motor velocities, 0: drive, 1: position
     */
    public double[] getVelocity() {
        return new double[] {motors[0].getSelectedSensorVelocity(), motors[1].getSelectedSensorVelocity()};
    }

    private double convertAngleToSetPoint(Rotation2d angle) {
        double targetTicks = (angle.getDegrees()+180)/360 * ConversionConstants.CTRE_TICKS_PER_REV;
        double currentTicks = motors[0].getSelectedSensorPosition() % ConversionConstants.CTRE_TICKS_PER_REV;
        double error = currentTicks - targetTicks;
        double rotations = 0;
        if (motors[0].getSelectedSensorPosition() >= 0) {
            rotations = Math.floor(motors[0].getSelectedSensorPosition() / ConversionConstants.CTRE_TICKS_PER_REV);
        } else {
            rotations = Math.ceil(motors[0].getSelectedSensorPosition() / ConversionConstants.CTRE_TICKS_PER_REV);
        }
        

        if (error < -ConversionConstants.CTRE_TICKS_PER_REV/2) {
            targetTicks -= ConversionConstants.CTRE_TICKS_PER_REV;
            // resetAngleEncoder(getCancoderAngle().getDegrees()/360 * ConversionConstants.CTRE_TICKS_PER_REV);
        } else if (error > ConversionConstants.CTRE_TICKS_PER_REV/2) {
            targetTicks += ConversionConstants.CTRE_TICKS_PER_REV; 
            // resetAngleEncoder(getCancoderAngle().getDegrees()/360 * ConversionConstants.CTRE_TICKS_PER_REV);
        } else {
            targetTicks += ConversionConstants.CTRE_TICKS_PER_REV * 0;
        }

        return targetTicks;
    }

    /**
     * set motor velocity and position from a state
     * @param desiredState SwerveModuleState, unoptimized, that corresponds to the swerve module
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState state =
            SwerveModuleState.optimize(
                desiredState, 
                Rotation2d.fromDegrees(getCancoderAngle().getDegrees()));
        
        double unitsVel = desiredState.speedMetersPerSecond / ConversionConstants.CTRE_NATIVE_TO_MPS;
        motors[0].set(TalonFXControlMode.Velocity, unitsVel);

        SmartDashboard.putNumber("ANGLESTATE", desiredState.angle.getRadians());
        double setpoint = (desiredState.angle.getDegrees()+180)/360 * ConversionConstants.CTRE_TICKS_PER_REV;
        SmartDashboard.putNumber("SETPOINT", setpoint);
        motors[1].set(TalonFXControlMode.Position, setpoint);
    }

    /**
     * reset encoders, setting both positions to 0
     */
    public void resetAngleEncoder(double i) {
        motors[1].setSelectedSensorPosition(i);
    }

    public void resetDriveEncoder(double i) {
        motors[0].setSelectedSensorPosition(i);
    }

    /**
     * update PID config of motors from PID Constants
     */
    public void updatePID() {
        motors[0].config_kF(PIDConstants.PID_LOOP_IDX, PIDConstants.DRIVE_GAINS_VELOCITY.F, PIDConstants.TIMEOUT_MS);
        motors[0].config_kP(PIDConstants.PID_LOOP_IDX, PIDConstants.DRIVE_GAINS_VELOCITY.P, PIDConstants.TIMEOUT_MS);
        motors[0].config_kI(PIDConstants.PID_LOOP_IDX, PIDConstants.DRIVE_GAINS_VELOCITY.I, PIDConstants.TIMEOUT_MS);
        motors[0].config_kD(PIDConstants.PID_LOOP_IDX, PIDConstants.DRIVE_GAINS_VELOCITY.D, PIDConstants.TIMEOUT_MS);
        motors[0].config_IntegralZone(PIDConstants.PID_LOOP_IDX, PIDConstants.DRIVE_GAINS_VELOCITY.IZONE, PIDConstants.TIMEOUT_MS);

        motors[1].config_kF(PIDConstants.PID_LOOP_IDX, PIDConstants.DRIVE_GAINS_POSITION.F, PIDConstants.TIMEOUT_MS);
        motors[1].config_kP(PIDConstants.PID_LOOP_IDX, PIDConstants.DRIVE_GAINS_POSITION.P, PIDConstants.TIMEOUT_MS);
        motors[1].config_kI(PIDConstants.PID_LOOP_IDX, PIDConstants.DRIVE_GAINS_POSITION.I, PIDConstants.TIMEOUT_MS);
        motors[1].config_kD(PIDConstants.PID_LOOP_IDX, PIDConstants.DRIVE_GAINS_POSITION.D, PIDConstants.TIMEOUT_MS);
        motors[0].config_IntegralZone(PIDConstants.PID_LOOP_IDX, PIDConstants.DRIVE_GAINS_POSITION.IZONE, PIDConstants.TIMEOUT_MS);
    }

    /**
     * initialize motor configs
     */
    private void initializeMotors() {
        for (int i = 0; i < motors.length; i++) {
            motors[i].configFactoryDefault();
            motors[i].set(ControlMode.PercentOutput, 0);
            motors[i].setNeutralMode(NeutralMode.Brake);
            motors[i].configNeutralDeadband(0.001);
            motors[i].setInverted(TalonFXInvertType.Clockwise);
        }
    }

    /**
     * initialize motor PID configs
     */
    private void initializeMotorsPID() {
        motors[0].configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
        PIDConstants.PID_LOOP_IDX, 
        PIDConstants.TIMEOUT_MS);
        motors[1].configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
        PIDConstants.PID_LOOP_IDX, 
        PIDConstants.TIMEOUT_MS);
        
  
  
        /* Config the peak and nominal outputs */
        motors[0].configNominalOutputForward(0, PIDConstants.TIMEOUT_MS);
        motors[0].configNominalOutputReverse(0, PIDConstants.TIMEOUT_MS);
        motors[0].configPeakOutputForward(PIDConstants.DRIVE_GAINS_VELOCITY.PEAK_OUTPUT, PIDConstants.TIMEOUT_MS);
        motors[0].configPeakOutputReverse(-PIDConstants.DRIVE_GAINS_VELOCITY.PEAK_OUTPUT, PIDConstants.TIMEOUT_MS);
  
        motors[1].configNominalOutputForward(0, PIDConstants.TIMEOUT_MS);
        motors[1].configNominalOutputReverse(0, PIDConstants.TIMEOUT_MS);
        motors[1].configPeakOutputForward(PIDConstants.DRIVE_GAINS_POSITION.PEAK_OUTPUT, PIDConstants.TIMEOUT_MS);
        motors[1].configPeakOutputReverse(-PIDConstants.DRIVE_GAINS_POSITION.PEAK_OUTPUT, PIDConstants.TIMEOUT_MS);
  
        /* Config the Velocity closed loop gains in slot0 */
        motors[0].config_kF(PIDConstants.PID_LOOP_IDX, PIDConstants.DRIVE_GAINS_VELOCITY.F, PIDConstants.TIMEOUT_MS);
        motors[0].config_kP(PIDConstants.PID_LOOP_IDX, PIDConstants.DRIVE_GAINS_VELOCITY.P, PIDConstants.TIMEOUT_MS);
        motors[0].config_kI(PIDConstants.PID_LOOP_IDX, PIDConstants.DRIVE_GAINS_VELOCITY.I, PIDConstants.TIMEOUT_MS);
        motors[0].config_kD(PIDConstants.PID_LOOP_IDX, PIDConstants.DRIVE_GAINS_VELOCITY.D, PIDConstants.TIMEOUT_MS);
        motors[0].config_IntegralZone(PIDConstants.PID_LOOP_IDX, PIDConstants.DRIVE_GAINS_VELOCITY.IZONE, PIDConstants.TIMEOUT_MS);
  
        motors[1].config_kF(PIDConstants.PID_LOOP_IDX, PIDConstants.DRIVE_GAINS_POSITION.F, PIDConstants.TIMEOUT_MS);
        motors[1].config_kP(PIDConstants.PID_LOOP_IDX, PIDConstants.DRIVE_GAINS_POSITION.P, PIDConstants.TIMEOUT_MS);
        motors[1].config_kI(PIDConstants.PID_LOOP_IDX, PIDConstants.DRIVE_GAINS_POSITION.I, PIDConstants.TIMEOUT_MS);
        motors[1].config_kD(PIDConstants.PID_LOOP_IDX, PIDConstants.DRIVE_GAINS_POSITION.D, PIDConstants.TIMEOUT_MS);
        motors[1].config_IntegralZone(PIDConstants.PID_LOOP_IDX, PIDConstants.DRIVE_GAINS_POSITION.IZONE, PIDConstants.TIMEOUT_MS);
  
        /* Hopefully make not continous */
        motors[1].configFeedbackNotContinuous(true, PIDConstants.TIMEOUT_MS);
        motors[1].configIntegratedSensorAbsoluteRange(AbsoluteSensorRange.Signed_PlusMinus180);
        motors[1].configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);

    }
    private void initializeCancoder() {
        cancoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        cancoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        cancoder.setPositionToAbsolute();
        cancoder.configSensorDirection(true);
    }
}
