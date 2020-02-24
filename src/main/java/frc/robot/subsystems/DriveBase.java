package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

//import frc.robot.sensors.Limelight;
import frc.robot.sensors.Pipeline;

public class DriveBase {
    //--------------------------------\\
    // DriveTrain Section:
    private CANSparkMax leftfrontmotor;
    private CANSparkMax leftrearmotor;
    private CANSparkMax rightfrontmotor;    
    private CANSparkMax rightrearmotor;
    //--------------------------------\\
    // Arm Section:
    private WPI_TalonSRX Arm;        // Will Likely Undergo Change.
    private WPI_TalonSRX Mouth;      // Will Likely Undergo Change.
    //--------------------------------\\
    // Climb Section:
    private WPI_TalonSRX Hook;       // Will Likely Undergo Change.
    private WPI_TalonSRX Climb_Motor1;// Will Likely Undergo Change.
    private WPI_TalonSRX Climb_Motor2;// Will Likely Undergo Change.
    //--------------------------------\\
    //Color Wheel Section:
    private WPI_TalonSRX Hand;        // Will Likely Undergo Change.
    private WPI_TalonSRX Spinner;     // Will Likely Undergo Change.
    //--------------------------------\\
    //private AHRS gyro = new AHRS();
    //--------------------------------\\
    // Constants:
    private final double pTurn = 0.005;
    private final double iTurn = 0.0;
    private final double dTurn = 0.0;

    private final double pDrive = 0.016;
    private final double iDrive = 0.0;
    private final double dDrive = 0.0;
    //--------------------------------\\
  
    // Controllers:
    private PIDController turnController = new PIDController(pTurn, iTurn, dTurn);
    private PIDController driveController = new PIDController(pDrive, iDrive, dDrive);
    private PIDController ballTurnController = new PIDController(pTurn, iTurn, dTurn);
    private PIDController ballDriveController = new PIDController(pDrive, iDrive, dDrive);

    public static final GenericHID.Hand left = GenericHID.Hand.kLeft;
    public static final GenericHID.Hand right = GenericHID.Hand.kRight;

    private XboxController controller = new XboxController(0);
    private XboxController controller1 = new XboxController(1); //Really Controller #2 But Base Zero so it is called "1".
    
    // Misc:
    private double tx_prev = 0;

    //private Limelight camera = new Limelight(Pipeline.BALL);

 

    public DriveBase(int leftfrontmotorPort, int leftrearmotorPort, int rightfrontmotorPort, int rightrearmotorPort) {
        this.leftfrontmotor = new CANSparkMax(leftfrontmotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);
        this.leftrearmotor = new CANSparkMax(leftrearmotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);
        this.rightfrontmotor = new CANSparkMax(rightfrontmotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);
        this.rightrearmotor = new CANSparkMax(rightrearmotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);
    }

    public void initialize() {

        leftfrontmotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        leftrearmotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        rightfrontmotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        rightrearmotor.setIdleMode(CANSparkMax.IdleMode.kCoast);

        leftfrontmotor.setOpenLoopRampRate(0.5);
        leftrearmotor.setOpenLoopRampRate(0.5);
        rightfrontmotor.setOpenLoopRampRate(0.5);
        rightrearmotor.setOpenLoopRampRate(0.5);

        leftfrontmotor.getEncoder().setPositionConversionFactor(34.19);
        leftrearmotor.getEncoder().setPositionConversionFactor(34.19);
        rightfrontmotor.getEncoder().setPositionConversionFactor(34.19);
        rightrearmotor.getEncoder().setPositionConversionFactor(34.19);

        turnController.setSetpoint(0);
        turnController.setTolerance(2);

        driveController.setSetpoint(0);
        driveController.setTolerance(5);

        ballTurnController.setSetpoint(0);
        ballTurnController.setTolerance(0.25);

        ballDriveController.setSetpoint(10);
        ballDriveController.setTolerance(0.5);

        reset();
    }

    public void reset() {

        leftfrontmotor.getEncoder().setPosition(0);
        leftrearmotor.getEncoder().setPosition(0);
        rightfrontmotor.getEncoder().setPosition(0);
        rightrearmotor.getEncoder().setPosition(0);

        //gyro.reset();

        turnController.setSetpoint(0);
        driveController.setSetpoint(0);

        tx_prev = 0;
    }

    public void arcadeDrive(XboxController controller, GenericHID.Hand left, GenericHID.Hand right) {
        double throttle = 0;
        double turn = 0;
        //-------------------------------------\\
        // Encoder Turn Values 
        // (Inverted to Account for diff sides.)
        leftfrontmotor.set(turn - throttle);
        leftrearmotor.set(turn - throttle);

        rightfrontmotor.set(turn + throttle);
        rightrearmotor.set(turn + throttle);
        //--------------------------------------\\
    }
    
    public void Shooter(XboxController controller1, GenericHID.Hand right) {
        // Controls: Xbox Controller (2)
        // Safety - Hold Right Trigger to Operate Further, thereby taking
        // safety off.
        // Intake

        // Values Defined:
        double intakeVelocity = 0.0;  // Value to be set after testing.
        double extakeVelocity = 0.0; // Value to be set after testing.
        double liftVelocity = 0.0;  // Value to be set after testing.
        double lowerVelocity = 0.0;// Value to be set after testing.

        // Operator Must Push Trigger, then Right Axis to Control Intake.
            // Failsafe was set since Operator is a Rookie.
        //-------------------------------------------\\

        // For controlling the Intake: 
        if (controller1.getTriggerAxis(right) > 0.5) {
            if (controller1.getAButtonPressed()){
                Mouth.set(intakeVelocity);
            } 
            // FailSafe (I-1):
            else { Mouth.set(0.0); }
        }
        // Failsafe (I-2):
        if (controller1.getAButtonReleased()) {
            Mouth.set(0.0);
            }
        // Failsafe (I-3):
        if (controller1.getTriggerAxis(right) < 0.5) {
            Mouth.set(0.0);
            }

        //-------------------------------------------\\
        // For controlling the Extake:
        if (controller1.getTriggerAxis(right) > 0.5) {
            if (controller1.getBButtonPressed()) {
                Mouth.set(extakeVelocity);
            }
            // FailSafe (E-1):
            else { Mouth.set(0.0); }
        }
        // Failsafe (E-2):
        if (controller1.getAButtonReleased()) {
            Mouth.set(0.0);
            }
        // Failsafe (E-3):
        if (controller1.getTriggerAxis(right) < 0.5) {
            Mouth.set(0.0);
            }
    
        //-------------------------------------------\\

        // For controlling the Arm (angle):

        //-------------------------------------------\\
            // Raise Up:
        if (controller1.getTriggerAxis(right) > 0.5) {
            if (controller1.getYButtonPressed()) {
                Arm.set(liftVelocity);
            } 
            // Failsafe (RU-1):
            else { Arm.set(0.0); }
        }
        // Failsafe (RU-2):
        if (controller1.getYButtonReleased()) {
            Arm.set(0.0);
            }
        // Failsafe (RU-3):
        if (controller1.getTriggerAxis(right) < 0.5) {
            Arm.set(0.0);
            }
        //-------------------------------------------\\
            // Lower Down:
        if (controller1.getTriggerAxis(right) > 0.5) {
            if (controller1.getXButtonPressed()) {
                Arm.set(lowerVelocity);
            }
            else
            // Failsafe (LD-1):
            { Arm.set(0.0); }
        }
        // Failsafe (LD-2):
        if (controller1.getYButtonReleased()) {
            Arm.set(0.0);
            }
        // Failsafe (LD-3):
        if (controller1.getTriggerAxis(right) < 0.5) {
            Arm.set(0.0);
            }
        }
        //-------------------------------------------\\
    
    public void Climber(XboxController controller1, GenericHID.Hand left) {
        double upperVelocity = 0.0; // Value to be set after testing.
        double lowerVelocity = 0.0; // Value to be set after testing.
        double climbUpVelocity = 0.25; // Value to be set after testing.
        double climbDownVelocity = 0.05; // Value to be set after testing.

        // Hoist Hook:
        if (controller1.getTriggerAxis(right) > 0.5) {
            if (controller1.getY(left) > 0.05) {
                Hook.set(upperVelocity);
            }
            if (controller1.getY(left) < 0.05) {
                Hook.set(lowerVelocity);
            }
        }
        // Failsafe (H-1):
        else
            { Hook.set(0.0); }
        // Failsafe (H-2):
        if (controller1.getTriggerAxis(right) < 0.05 || controller1.getTriggerAxis(right) > 0.05) {
            Hook.set(0.0);
            }

        // Climb Up:
        if (controller1.getTriggerAxis(right) > 0.05) {
            if (controller.getY(right) > 0.05) {
                Climb_Motor1.set(climbUpVelocity);  // "throttle = controller.getY(left);"  Keep this in mind!!!
                Climb_Motor2.set(climbUpVelocity);
            }
            if (controller.getY(right) < -0.05) {
                Climb_Motor1.set(climbDownVelocity);
                Climb_Motor2.set(climbDownVelocity);    
            } 
        // Failsafe (1):
        else {
            Climb_Motor1.set(0.0);
            Climb_Motor2.set(0.0);
            }
        }
        // Failsafe (2):
        if (controller1.getTriggerAxis(right) < 0.05 || controller1.getTriggerAxis(right) > -0.05) {
            Climb_Motor1.set(0.0);
            Climb_Motor2.set(0.0);
        }
    }

    public void arcadeTuning(XboxController controller, GenericHID.Hand left) {
        //double turn = turnController.calculate(getAngle());
        double throttle = 0;
        double turn = 0;

        // Stutter Stepping with controller.
        //  Plus or minus 0.05 Joystick Axis trigger causes automatic rotation. 02/23/20

        // Arcade Drive (1):
        if (controller.getY(left) > 0.05 || controller.getY(left) < -0.05) {
            throttle = controller.getY(left);
            throttle = -throttle;  //to southpaw or not.  currently not.
        
        // Failsafe (1):
        } else { 
                throttle = 0.0; 
                turn = 0.0;
                }

        // Arcade Drive (2):
        if (controller.getX(left) > 0.05 || controller.getX(left) < -0.05) {
            turn = controller.getX(left);
        
        // Failsafe (2):
        } else { 
            turn = 0.0; 
            throttle = 0.0;
                }

        // Failsafe (General):
        if (controller.getY(left) < 0.05 || controller.getY(left) > -0.05) {
            throttle = 0;
            }
        }
        
    public void distanceDrive() {
        double throttle = driveController.calculate(getLeftPosition());
        // Inverting the motors since we don't want to spin 
        // when we're supposed to drive forward.
        leftfrontmotor.set(-throttle);
        leftrearmotor.set(-throttle);

        rightfrontmotor.set(throttle);
        rightrearmotor.set(throttle);
    }

    // This is not necessary due to the lack of a gyro. 02/23/20
    /*  
    public void setAngle(double angle) {
        turnController.setSetpoint(angle);
    }
    */

    public void setDistance(double degrees) {
        driveController.setSetpoint(degrees);
    }

    //--------------------------------------------------\\
    // Distance (x) Section: 
    public double getLeftPosition() {
        return leftfrontmotor.getEncoder().getPosition();   
    }

    public double getRightPosition() {
        return rightfrontmotor.getEncoder().getPosition();
    }
    //--------------------------------------------------\\


    //--------------------------------------------------\\
    // intakeVelocity (dx/dt) Section: 
    public double getLeftVelocity() {
        return leftfrontmotor.getEncoder().getVelocity();
    }

    public double getRightVelocity() {
        return rightfrontmotor.getEncoder().getVelocity();
    }
    //--------------------------------------------------\\


    /*
    public double getAngle() {
        return gyro.getAngle();
    }
    */


    //------------------------------------------\\
    // Targeting Section:
    public boolean turnOnTarget() {
        return turnController.atSetpoint();
    }
    public boolean driveOnTarget() {
        return driveController.atSetpoint();
    }
    public boolean ballTurnOnTarget() {
        return ballTurnController.atSetpoint();
    }
    public boolean ballDriveOnTarget() {
        return ballDriveController.atSetpoint();
    }
    //------------------------------------------\\



    public void dashboard() {
        // Extra  Data Display:
        SmartDashboard.putData("Turn Controller", turnController);
        SmartDashboard.putData("Drive Controller", driveController);
        SmartDashboard.putData("Ball Turn Controller", ballTurnController);
        SmartDashboard.putData("Ball Drive Controller", ballDriveController);

        SmartDashboard.putNumber("Left Position", getLeftPosition());
        SmartDashboard.putNumber("Right Position", getRightPosition());
        SmartDashboard.putNumber("Left intakeVelocity", getLeftVelocity());
        SmartDashboard.putNumber("Right intakeVelocity", getRightVelocity());

        //SmartDashboard.putNumber("Angle", getAngle());

        //SmartDashboard.putNumber("tv", camera.getValues()[0]);
        //SmartDashboard.putNumber("tx", camera.getValues()[1]);
        //SmartDashboard.putNumber("ta", camera.getValues()[2]);
    }
}