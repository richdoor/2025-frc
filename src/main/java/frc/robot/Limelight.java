package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Limelight {
    
    final double limelightHeight, targetHeight, limelightAngle;
    double xOff, yOff, areaValue, validity, targetDistance;

    public static NetworkTable limelightTable;
    NetworkTableEntry xOffEntry, yOffEntry, areaValueEntry, validityEntry;

    /**
     * Constructs a limelight object
     * @param limelightHeight Vertical distance between the limelight and the ground in inches
     * @param targetHeight Verticl distance between the target and the ground in inches
     * @param limelightAngle Angle between the limelight and level (from limelight's height)
     */
    public Limelight (double limelightHeight, double targetHeight, double limelightAngle) {

        this.limelightHeight = limelightHeight;
        this.targetHeight = targetHeight;
        this.limelightAngle = limelightAngle;

        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        xOffEntry = limelightTable.getEntry("tx");
        yOffEntry = limelightTable.getEntry("ty");
        areaValueEntry = limelightTable.getEntry("ta");
        validityEntry = limelightTable.getEntry("tv");
        


    }

    /**
     * Periodically updates the limelight variables
     * @param smartDashboardDisplay Boolean to display whether or not to display smart dashboard values
     * @return Returns the variable yOff for use
     */
    public double updateLimelightVariables (Boolean smartDashboardDisplay) {

        xOff = xOffEntry.getDouble(0.0);
        yOff = yOffEntry.getDouble(0.0);
        areaValue = areaValueEntry.getDouble(0.0);
        validity = validityEntry.getDouble(0.0);
        //targetDistance = (targetHeight - limelightHeight) / Math.tan((limelightAngle + yOff) * Math.PI / 180);
        
        //if (smartDashboardDisplay) {
        //    SmartDashboard.putNumber("X Offset", xOff);
        //    SmartDashboard.putNumber("Y Offset", yOff);
         //   SmartDashboard.putNumber("Area (Target Size)", areaValue);
         //   SmartDashboard.putNumber("Validity", validity);
        //}

        
        return yOff;

    }
    
}