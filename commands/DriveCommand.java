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
    private final DriveSubsystem drive;
    private final XboxController controller;
    private double forward;
    private double rot;
    
    public DriveCommand(DriveSubsystem subsystem, XboxController controller) {
        this.drive = subsystem;
        addRequirements(this.drive);
        this.controller = controller;
    }


    private void drive() {
        double vx = this.controller.getLeftX() * DriveConstants.LIMIT_VX;
        double vy = this.controller.getLeftY() * DriveConstants.LIMIT_VY;
        double rot = this.controller.getRightX() * Math.PI * DriveConstants.LIMIT_ROT;

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
        // System.out.println(_drive.getErrorStates()[2][1]);
        // System.out.println(_drive.getErrorStates()[2][0]);

        this.drive.updateSpeeds(rot, vx, vy);
        this.drive.updateModuleStates();

        for(int i = 0; i < this.drive.getStatesLength(); i++) {
            this.drive.driveFromOptimizedState(i);
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
        drive();
    }
}
