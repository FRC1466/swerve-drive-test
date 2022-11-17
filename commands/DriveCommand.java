package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PIDConstants;
import frc.robot.subsystems.DriveSubsystem;

import java.lang.Math;

public class DriveCommand extends CommandBase {
    private final DriveSubsystem m_drive;
    private final XboxController m_controller;
    private boolean m_isFieldRelative;
    private double vx = 0;
    private double vy = 0;
    private double rot = 0;
    
    /**
     * Default command for driving
     * @param subsystem drive subsystem
     * @param controller drive controller
     */
    public DriveCommand(
        DriveSubsystem subsystem, 
        XboxController controller,
        boolean isFieldRelative
        ) {
        m_drive = subsystem;
        addRequirements(m_drive);
        m_controller = controller;
        m_isFieldRelative = isFieldRelative;
    }

    /**
     * local driving function
     */
    private void m_drive() {
        vx = m_controller.getLeftX() * DriveConstants.LIMIT_VX;
        vy = m_controller.getLeftY() * DriveConstants.LIMIT_VY;
        rot = m_controller.getRightX() * DriveConstants.LIMIT_ROT;

        if (!(Math.abs(vx) > 0.15)) {
            vx = 0;
        }

        if (!(Math.abs(vy) > 0.15)) {
            vy = 0;
        }

        if (!(Math.abs(rot) > 0.1)) {
            rot = 0;
        }

        if (m_isFieldRelative) {
            m_drive.updateSpeedsFieldRelative(rot, vx, vy);
        } else {
            m_drive.updateSpeeds(rot, vx, vy);
        }
        m_drive.updateModuleStates();
        m_drive.drive();
        
    }

    /**
     * update smartdashboard with a few values
     */
    private void updateSmartDashboard() {
        SmartDashboard.putNumber("vx", vx);
        SmartDashboard.putNumber("vy", vy);
        SmartDashboard.putNumber("rot", rot);

        SmartDashboard.putNumber("vx limit", DriveConstants.LIMIT_VX);
        SmartDashboard.putNumber("vy limit", DriveConstants.LIMIT_VY); 
        SmartDashboard.putNumber("rot limit", DriveConstants.LIMIT_ROT);

        DriveConstants.LIMIT_VX = SmartDashboard.getNumber("vx limit", DriveConstants.LIMIT_VX);
        DriveConstants.LIMIT_VY = SmartDashboard.getNumber("vy limit", DriveConstants.LIMIT_VY); 
        DriveConstants.LIMIT_ROT = SmartDashboard.getNumber("rot limit", DriveConstants.LIMIT_ROT);
    }

    /**
     * Update PID system from SmartDashboard for quick and easy tuning
     */
    private void updatePID() {
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

        m_drive.updatePID();

    }
    

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_drive();
        // updateSmartDashboard();
        // updatePID();
    }
}
