// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class ConversionConstants {
    public static final double
      GEAR_RATIO = 8.14,
      CTRE_TICKS =  2048,
      CTRE_TICKS_PER_REV = CTRE_TICKS * GEAR_RATIO,
      WHEEL_DIAMETER = 1.975*2, //inches
      CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI,
      INCHES_PER_TICK = CIRCUMFERENCE / CTRE_TICKS_PER_REV,
      IPDS_TO_MPH = 0.568,
      IPDS_TO_MEPS = 0.254,
      METERS_PER_TICK = INCHES_PER_TICK / 37.3701,
      CTRE_NATIVE_TO_MPH = INCHES_PER_TICK * IPDS_TO_MPH,
      CTRE_NATIVE_TO_MPS = INCHES_PER_TICK * IPDS_TO_MEPS;

  }

  public static final class DriveConstants {
    public static final int 
      FRONTRIGHT_PORT_DRIVE = 3,
      FRONTRIGHT_PORT_ROTATE = 4,
      FRONTLEFT_PORT_DRIVE = 1,
      FRONTLEFT_PORT_ROTATE = 2,
      BACKRIGHT_PORT_DRIVE = 5,
      BACKRIGHT_PORT_ROTATE = 6,
      BACKLEFT_PORT_DRIVE = 7,
      BACKLEFT_PORT_ROTATE = 8;

    public static final double 
      TRACKWIDTH_METERS = 0.375;

    // Drive limiters

    public static final double 
      LIMIT_VX = 1.0,
      LIMIT_VY = 1.0,
      LIMIT_ROT = 1.0;


  }

  public static final class OIConstants {
    public static final int 
      DRIVER_PORT = 0,
      INTAKE_PORT = 1;
  }

  public static final class AutoConstants {

  }

  public static final class IntakeConstants {

  }

  public static final class PIDConstants {
    public static final int 
      SLOT_IDX = 0,
      PID_LOOP_IDX = 0,
      TIMEOUT_MS = 30;

    //                                                    kP   	 kI    kD      kF          Iz    PeakOut
    public final static Gains DRIVE_GAINS_VELOCITY  = new Gains(0.2, 0.0001, 4.0, 0,  0,  0.6);
    //   0.35,0.001,0.2                                           kP: 4   	 kI    kD      kF          Iz    PeakOut
    public final static Gains DRIVE_GAINS_POSITION  = new Gains(0.05, 0.00001, 0, 0,  0,  0.25);
  }

}
