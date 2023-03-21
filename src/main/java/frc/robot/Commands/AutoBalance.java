package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Constants.DriveConstants;
public class AutoBalance extends CommandBase {
    private DriveSubsystem driveSubsystem;
    public void AutoBalance(DriveSubsystem driveSubsystem){
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
    }
    @Override
    public void initialize(){

    }
    @Override
    public void execute(){
        double xAxisRate;
        double pitchAngleDegrees    = driveSubsystem.getPitch();
        boolean autoBalanceXMode = true;

        
        if ((!autoBalanceXMode && Math.abs(pitchAngleDegrees) >= Math.abs(DriveConstants.kOffBalanceAngleThresholdDegrees))) {
            autoBalanceXMode = true;
        }
        else if ( autoBalanceXMode && 
                  (Math.abs(pitchAngleDegrees) <= 
                   Math.abs(DriveConstants.kOnBalanceAngleThresholdDegrees))) {
            autoBalanceXMode = false;
                   }
        
        // Control drive system automatically, 
        // driving in reverse direction of pitch/roll angle,
        // with a magnitude based upon the angle
        
        if ( autoBalanceXMode ) {
            double pitchAngleRadians = pitchAngleDegrees * (Math.PI / 180.0);
            xAxisRate = Math.sin(pitchAngleRadians) * -1;
        }
        
        double pitchAngleRadians = pitchAngleDegrees * (Math.PI / 180.0);  
        driveSubsystem.tankDriveIK(pitchAngleDegrees, pitchAngleDegrees, autoBalanceXMode);
    }
        
}
