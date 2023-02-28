
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

public class Autos extends SequentialCommandGroup {
    /**
     * Creates a new ComplexAuto.
     *
     * @param drive The drive subsystem this command will run on
     * @param  arm The arm subsystem this command will run on
     */
    public Autos(DriveSubsystem drive, ArmSubsystem arm) {
      addCommands(
          // Drive forward the specified distance
        
          );
    }
  
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
