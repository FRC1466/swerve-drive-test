package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import java.lang.Math;

public class DriveCommand extends CommandBase {
    private final DriveSubsystem m_drive;
    private final XboxController m_controller;
    private int limitIter = 0;
    private boolean isLimit = false;
    private int stopIter = 0;
    private int listIter = 0;
    private double pastForward[] = {0, 0, 0, 0, 0};
    private double forward;
    private double rot;
    
    public DriveCommand(DriveSubsystem subsystem, XboxController controller) {
        m_drive = subsystem;
        addRequirements(m_drive);
        m_controller = controller;
    }


    private void m_drive() {
        double vx = m_controller.getRightX() * 0.5;
        double vy = m_controller.getLeftY() * 0.5;
        double rot = 0; // m_controller.getRightX() * Math.PI;

        if (!(Math.abs(vx) > 0.15)) {
            vx = 0;
        }

        if (!(Math.abs(vy) > 0.15)) {
            vy = 0;
        }

        if (!(Math.abs(rot) > 0.1)) {
            rot = 0;
        }

        // System.out.println("rot: " + rot);
        // System.out.println("vx: " + vx);
        // System.out.println("vy: " + vy);
        // System.out.println(m_drive.getErrorStates()[2][1]);
        // System.out.println(m_drive.getErrorStates()[2][0]);

        m_drive.updateSpeeds(rot, vx, vy);
        m_drive.updateModuleStates();

        for(int i = 0; i < m_drive.getStatesLength(); i++) {
            m_drive.driveFromOptimizedState(i);
        }
        
    }

    private void updateSmartDashboard() {
        SmartDashboard.putNumber("forward", forward);
        SmartDashboard.putNumber("rotation", rot);
    }

    private void updateSmartDashboardPID() {
        SmartDashboard.putNumber("forward", forward);
        SmartDashboard.putNumber("rotation", rot);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        m_drive();
    }
}
