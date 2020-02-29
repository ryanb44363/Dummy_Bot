package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
//import frc.robot.sensors.ColorSensor;
import frc.robot.subsystems.DriveBase;


public class Robot extends TimedRobot {
    
    public static final XboxController.Hand kLeft = XboxController.Hand.kLeft;
    public static final XboxController.Hand kRight = XboxController.Hand.kRight;

    public static XboxController controller = new XboxController(0);  //Drive controller is on zero.

    public static DriveBase base = new DriveBase(1, 2, 3, 5);
    // During the testing of the Motor Controllers, "4" became "5", which should not cause any problems.

    //public class kLeft = DriveBase(1,2);
    //public static DistanceSensor dist = new DistanceSensor(Rev2mDistanceSensor.Port.kOnboard);

    //public static ColorSensor color = new ColorSensor(I2C.Port.kMXP);

    @Override
    public void robotInit() {
        base.initialize();
        //dist.initialize();
    }

    @Override
    public void teleopInit() {
        base.initialize();
        base.reset();
    }

    @Override
    public void teleopPeriodic() {
        reseter();

        //setAngle(); This should not change anything for our build since it we are not yet running with a gyro. 02/23/20
        
        drive();

        dashboard();
    }

    public void drive() {
        if (controller.getTriggerAxis(Hand.kRight) > 0.5) {
            //base.arcadeTuning(controller, left);
            base.arcadeTuning(controller, Hand.kLeft);  
            // Should include both left and right.
        /*
        } else if (controller.getTriggerAxis(left) > 0.5) {
            base.ballFollowDrive();
        */
        } else if (controller.getBumper(Hand.kLeft)) {
            base.distanceDrive();
        } else {
            base.arcadeDrive(controller, Hand.kLeft, Hand.kRight);
            // Should include both left and right.
        }

    }

    public void dashboard() {
        base.dashboard();
        //dist.dashboard();
        //color.dashboard();
    }

    public void reseter() { 
    }

}
