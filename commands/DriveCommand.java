package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
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

         /* m_drive.resetAngleByCancoderOffset(
            new double[] {
                SmartDashboard.getNumber("0 cc", 0.0),
                SmartDashboard.getNumber("1 cc", 0.0),
                SmartDashboard.getNumber("2 cc", 0.0),
                SmartDashboard.getNumber("3 cc", 0.0),
            }
        ); */
        SmartDashboard.putNumber("vx limit", DriveConstants.LIMIT_VX);
        SmartDashboard.putNumber("vy limit", DriveConstants.LIMIT_VY); 
        SmartDashboard.putNumber("rot limit", DriveConstants.LIMIT_ROT);
        
    }

    /**
     * local driving function
     */
    private void m_drive() {
        vx = m_controller.getLeftX() * DriveConstants.LIMIT_VX;
        vy = -m_controller.getLeftY() * DriveConstants.LIMIT_VY;
        rot = -m_controller.getRightX() * DriveConstants.LIMIT_ROT;

        if (!(Math.abs(vx) > 0.15)) {
            vx = 0;
        }

        if (!(Math.abs(vy) > 0.15)) {
            vy = 0;
        }

        if (!(Math.abs(rot) > 0.16)) {
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

        DriveConstants.LIMIT_VX = SmartDashboard.getNumber("vx limit", DriveConstants.LIMIT_VX);
        DriveConstants.LIMIT_VY = SmartDashboard.getNumber("vy limit", DriveConstants.LIMIT_VY); 
        DriveConstants.LIMIT_ROT = SmartDashboard.getNumber("rot limit", DriveConstants.LIMIT_ROT);
    }

    /**
     * Update PID system from SmartDashboard for quick and easy tuning
     */
    private void updatePID() {
        m_drive.updatePIDConstants();
        m_drive.updatePIDConfigs();
    }
    

    @Override
    public void initialize() {
    
    }

    @Override
    public void execute() {
        m_drive();
        updateSmartDashboard();
        // updatePID();
        
        
    }
}
