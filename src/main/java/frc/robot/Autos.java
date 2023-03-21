
package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.HashMap;
import java.util.List;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.*;

public class Autos {
    private DriveSubsystem drive;
	private ArmSubsystem arm;
	private IntakeSubsystem intake;
	/**
     * Creates a new ComplexAuto.
     *
     * @param drive The drive subsystem this command will run on
     * @param  arm The arm subsystem this command will run on
     */
    public Autos(DriveSubsystem drive){//, ArmSubsystem arm, IntakeSubsystem intake) {
		this.drive = drive;
		this.intake = intake;
		this.arm = arm;
    }
	public Command driveBack (){
		// /return drive.driveTo(132.75-28.5+38, 1).withTimeout(4);
		return Commands.sequence(
			drive.run(()-> drive.tankDrive(1, 1)).withTimeout(0.1),
			drive.run(()->drive.tankDrive(-0.4, -0.4)).withTimeout(1),
			drive.run(()-> drive.tankDrive(0, 0))
			);
		//return drive.startEnd(()-> drive.tankDrive(0.4, 0.4), ()-> drive.tankDrive(0, 0)).withTimeout(3);
	}
	public Command autoBalanceForward (){
		return Commands.sequence(
			Commands.sequence(
				//end of comunity(short length) - robot lenth + 1/2 of charge station length
				drive.driveTo(132.75+10, 1).withTimeout(4),
				drive.autoBalanceCommand().withTimeout(5),
				drive.stop()
			)
		);
	}
		/*
	//possition robot facing grid
	public Command cubeAndDriveCommand (){
		return Commands.sequence(
			Commands.sequence(
				Commands.parallel(
					drive.driveTo(2, 0.75),
					intake.holdCube()
				),
				arm.raiseArm().withTimeout(2),
				intake.intakeCone().withTimeout(2),
				Commands.parallel(
					drive.driveTo(-132.75, 0.75),
					intake.disable()
				)
				
			)
		);
	}
	//possition robot facing grid
	public Command coneAndDriveCommand (){
			return Commands.sequence(
				Commands.sequence(
					Commands.parallel(
						drive.driveTo(2, 0.75),
						intake.holdCone()
					),
					arm.raiseArm().withTimeout(2),
					intake.intakeCube().withTimeout(2),
					Commands.parallel(
						drive.driveTo(-132.75, 0.75),
						intake.disable()
					)
					
				)
			);
		}
	public Command autoBalance (){
		return Commands.sequence(
			Commands.sequence(
				//end of comunity(short length) - robot lenth + 1/2 of charge station length
				drive.driveTo(132.75-28.5+38, 0.75),
				drive.autoBalanceCommand().withTimeout(5),
				drive.stop()
			)
		);
	}*/

  
}
    /*
	// used by pathplanner
	public static final PIDController ppXController = new PIDController(autoDriveKp, autoDriveKi, autoDriveKd);
	public static final PIDController ppYController = new PIDController(autoDriveKp, autoDriveKi, autoDriveKd);
	public static final PIDController ppRotationController = new PIDController(autoAngleKp, autoAngleKi, autoAngleKd); // rad
	static {
		ppRotationController.enableContinuousInput(-Math.PI, Math.PI);
	}

	public static Command simpleWall(Swerve swerve) {
		List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("simple wall",
			new PathConstraints(maxVelocity_mps, maxAcceleration_mps2));

		if (pathGroup == null)
			return new PrintCommand("no path group");

		HashMap<String, Command> eventMap = new HashMap<>();
		eventMap.put("start", new PrintCommand("put block down"));

		return new FollowPathWithEvents(
			new PathPlannerCommand(pathGroup.get(0), swerve, true, true),
			pathGroup.get(0).getMarkers(),
			eventMap);
	}
*/
