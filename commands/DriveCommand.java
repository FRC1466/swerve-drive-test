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
    private double vx = 0;
    private double vy = 0;
    private double rot = 0;
    
    /**
     * Default command for driving
     * @param subsystem drive subsystem
     * @param controller drive controller
     */
    public DriveCommand(DriveSubsystem subsystem, XboxController controller) {
        m_drive = subsystem;
        addRequirements(m_drive);
        m_controller = controller;
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

        m_drive.updateSpeeds(rot, vx, vy);
        m_drive.updateModuleStates();

        for(int i = 0; i < m_drive.getStatesLength(); i++) {
            m_drive.driveFromOptimizedState(i);
        }
        
    }

    /**
     * update smartdashboard with a few values
     */
    private void updateSmartDashboard() {
        SmartDashboard.putNumber("vx", vx);
        SmartDashboard.putNumber("vy", vy);
        SmartDashboard.putNumber("rot", rot);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        m_drive();
        updateSmartDashboard();
    }
}
