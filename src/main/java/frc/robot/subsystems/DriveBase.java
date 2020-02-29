package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.robot.sensors.ColorSensor;
import frc.robot.subsystems.DriveBase;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.command.PIDSubsystem;

//import frc.robot.sensors.Limelight;
import frc.robot.sensors.Pipeline;

public class DriveBase {
    //--------------------------------\\
    // DriveTrain Section:
    public CANSparkMax leftfrontmotor;  // Should be motor controller #1.
    public CANSparkMax leftrearmotor;   // Should be motor controller #2.
    public CANSparkMax rightfrontmotor; // Should be motor controller #3.   
    public CANSparkMax rightrearmotor;  // Should be motor controller #4.
    //--------------------------------\\
    //public AHRS gyro = new AHRS();
    //--------------------------------\\
    // Constants:
    public final double pTurn = 0.005;
    public final double iTurn = 0.0;
    public final double dTurn = 0.0;

    public final double pDrive = 0.016;
    public final double iDrive = 0.0;
    public final double dDrive = 0.0;
    //--------------------------------\\
  
    // Controllers:
    public PIDController turnController = new PIDController(pTurn, iTurn, dTurn);
    public PIDController driveController = new PIDController(pDrive, iDrive, dDrive);
    public PIDController ballTurnController = new PIDController(pTurn, iTurn, dTurn);
    public PIDController ballDriveController = new PIDController(pDrive, iDrive, dDrive);

    public static final XboxController.Hand kLeft = XboxController.Hand.kLeft;
    public static final XboxController.Hand kRight = XboxController.Hand.kRight;

    public XboxController controller = new XboxController(0);
    //public XboxController controller1 = new XboxController(1); //Really Controller #2 But Base Zero so it is called "1".
 
    // Misc:
    public double tx_prev = 0;

    //public Limelight camera = new Limelight(Pipeline.BALL);

 

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
                        // Should not need to define left and right for one controller.
    public void arcadeDrive(XboxController controller, XboxController.Hand left, XboxController.Hand right) {
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
             
    public void arcadeTuning(XboxController controller, Hand left) {
        //double turn = turnController.calculate(getAngle());
        double throttle = 0;
        double turn = 0;

        // Stutter Stepping with controller.
        //  Plus or minus 0.05 Joystick Axis trigger causes automatic rotation. 02/23/20

        // Arcade Drive (1):
        if (controller.getY(Hand.kLeft) > 0.05 || controller.getY(Hand.kLeft) < -0.05) {
            throttle = controller.getY(Hand.kLeft);
            throttle = -throttle;  //to southpaw or not.  currently not.
        
        // Failsafe (1):
        } else { 
                throttle = 0.0; 
                turn = 0.0;
                }

        // Arcade Drive (2):
        if (controller.getX(Hand.kLeft) > 0.05 || controller.getX(Hand.kLeft) < -0.05) {
            turn = controller.getX(Hand.kLeft);
        
        // Failsafe (2):
        } else { 
            turn = 0.0; 
            throttle = 0.0;
                }

        // Failsafe (General):
        if (controller.getY(Hand.kLeft) < 0.05 || controller.getY(Hand.kLeft) > -0.05) {
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
