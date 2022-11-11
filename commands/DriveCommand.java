package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

import java.lang.Math;

public class DriveCommand extends CommandBase {
    private final DriveSubsystem drive;
    private final XboxController controller;
    private double vx;
    private double vy;
    private double rot;
    
    /**
     * Initialize a basic drive command
     * @param drive the drive subsystem
     * @param controller the drive controller
     */
    public DriveCommand(DriveSubsystem drive, XboxController controller) {
        this.drive = drive;
        addRequirements(this.drive);
        this.controller = controller;
    }


    private void drive() {
        vx = this.controller.getLeftX() * DriveConstants.LIMIT_VX;
        vy = this.controller.getLeftY() * DriveConstants.LIMIT_VY;
        rot = this.controller.getRightX() * Math.PI * DriveConstants.LIMIT_ROT;

        if (!(Math.abs(vx) > 0.15)) {
            vx = 0;
        }

        if (!(Math.abs(vy) > 0.15)) {
            vy = 0;
        }

        if (!(Math.abs(rot) > 0.1)) {
            rot = 0;
        }

        this.drive.updateSpeeds(rot, vx, vy);
        this.drive.updateModuleStates();

        for(int i = 0; i < this.drive.getStatesLength(); i++) {
            this.drive.driveFromOptimizedState(i);
        }
        
    }

    private void updateSmartDashboardSpeeds() {
        SmartDashboard.putNumber("horizontal velocity", vx);
        SmartDashboard.putNumber("vertical velocity", vy);
        SmartDashboard.putNumber("rotation", rot);
    }


    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        drive();
        updateSmartDashboardSpeeds();
    }
}
