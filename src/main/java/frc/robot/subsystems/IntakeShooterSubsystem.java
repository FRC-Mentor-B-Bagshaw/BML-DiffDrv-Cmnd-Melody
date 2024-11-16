package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;//https://software-metadata.revrobotics.com/REVLib-2024.json
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class IntakeShooterSubsystem extends SubsystemBase {
    private final Spark intakeRollerMotor = new Spark(2);
    private CANSparkMax Shooter_R = new CANSparkMax(24, MotorType.kBrushless);
    private CANSparkMax Shooter_L = new CANSparkMax(23, MotorType.kBrushless);
    private CANSparkMax Loader_R = new CANSparkMax(22, MotorType.kBrushless);
    private CANSparkMax Loader_L = new CANSparkMax(21, MotorType.kBrushless);
    private CANSparkMax intakeNeo = new CANSparkMax(3, MotorType.kBrushless);  
   
    public static double intakeSpeed = 0;
    public static double MaxIntakeSpeed = 1;
    public static double loaderSpeed = 0;
    public static double shooterSpeed = 0;
    public static double roboTimer = 0;
    public static boolean hasTarget;


  

    public IntakeShooterSubsystem(){
        Loader_L.setIdleMode(IdleMode.kBrake);
        Loader_R.setIdleMode(IdleMode.kBrake);
        
    }

/**
   * Sets the desired intake and shooter states
   *
   * @param intake is the speed of the intake motors
   * @param loaderOn turns on the intake motors
   * @param shooterOn turns on the intake motors
   * 
   */
  public void setNoteMotors( Double intake, Boolean loaderOn, Boolean shooterOn) {
    intakeSpeed = intake;    
    loaderSpeed = loaderOn ? 1 : 0 ;
    shooterSpeed = shooterOn ? 1 : 0;
        
        
  }

    @Override
    public void periodic() {
        Loader_L.set(loaderSpeed);
        Loader_R.set(-loaderSpeed);
        Shooter_L.set(shooterSpeed);
        Shooter_R.set(-shooterSpeed);
    
        intakeNeo.set(intakeSpeed * 0.25);
        intakeRollerMotor.set(-intakeSpeed);
    }



}
