package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants.Cargo;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class IntakeSubsystem extends SubsystemBase {
    private final CANSparkMax armShoot = new CANSparkMax(ArmConstants.kIntakeSparkMaxCANID, MotorType.kBrushless);
    private Cargo last = Cargo.NOTHING;

    public IntakeSubsystem (){
        armShoot.set(0);
    }
    public void set(double intake_value){
        // - shoot cargo; + intake cargo
        armShoot.set(intake_value);
    }
    /**
     * Command to intake a cube or eject a cone
     * @return 
     */
    public Command intakeCube () {
        //last = Cargo.CUBE;
        return this.run(() -> this.set(ArmConstants.INTAKE_OUTPUT_POWER));
    }
    /**
     * holds cube
     * @return
     */
    public Command holdCube () {
        //last = Cargo.CUBE;
        return this.run(() -> this.set(ArmConstants.INTAKE_HOLD_POWER));
    }
    /**
     * Command to intake a cone or eject a cube
     * @return 
     */
    public Command intakeCone () {
        return this.startEnd(() -> this.set(-ArmConstants.INTAKE_OUTPUT_POWER), () -> this.set(-ArmConstants.INTAKE_HOLD_POWER));
    }
    /**
     * holds Cone
     * @return
     */
    public Command holdCone () {
        //last = Cargo.CUBE;
        return this.run(() -> this.set(-ArmConstants.INTAKE_HOLD_POWER));
    }
    /**
     * stops intake
     * @return
     */
    public Command disable( ){
        return this.run(() -> this.set(0));
    }
          
}

