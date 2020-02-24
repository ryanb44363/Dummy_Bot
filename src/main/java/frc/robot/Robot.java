package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

import frc.robot.sensors.ColorSensor;
import frc.robot.subsystems.DriveBase;

public class Robot extends TimedRobot {

    public static XboxController controller = new XboxController(0);
    public static final GenericHID.Hand left = GenericHID.Hand.kLeft;
    public static final GenericHID.Hand right = GenericHID.Hand.kRight;

    public static DriveBase base = new DriveBase(1, 2, 3, 4);

    //public static DistanceSensor dist = new DistanceSensor(Rev2mDistanceSensor.Port.kOnboard);

    public static ColorSensor color = new ColorSensor(I2C.Port.kMXP);

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
        if (controller.getTriggerAxis(right) > 0.5) {
            base.arcadeTuning(controller, left);
        /*
        } else if (controller.getTriggerAxis(left) > 0.5) {
            base.ballFollowDrive();
        */
        } else if (controller.getBumper(left)) {
            base.distanceDrive();
        } else {
            base.arcadeDrive(controller, left, right);
        }

    }

    public void dashboard() {
        base.dashboard();
        //dist.dashboard();
        color.dashboard();
    }

    public void reseter() { 
    }

}