package org.firstinspires.ftc.teamcode.Libraries.MMLib.Utils;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Utils.Configuration;

/**
 * this imu class represents the imu of the newer control hubs (BHI260IMU) and not the older ones (BNO055IMU)
 * if you're using the older imu, switch to {@link com.qualcomm.hardware.bosch.BNO055IMUNew BNO055IMUNew}
 */
public class MMIMU {

    BHI260IMU imu;

    double yawOffset = 0;

    /**
     * here u can define the orientation of the control hub, in respect to the robot.
     */
    public MMIMU(HardwareMap hardwareMap) {
        imu = hardwareMap.get(BHI260IMU.class, Configuration.IMU);
        BHI260IMU.Parameters imuParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, //right here!
                        RevHubOrientationOnRobot.UsbFacingDirection.UP //and here!
                )
        );
        imu.initialize(imuParameters);
    }

    /**
     * this method returns the yaw of the robot, while respecting a certain offset.
     * @return the yaw of the robot
     */
    public double getYawInDegrees() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) + yawOffset;
    }

    /**
     * set a new yaw to the robot
     * @param newYaw the new yaw
     */
    public void setYaw(double newYaw) {
        yawOffset = newYaw - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    /**
     * reset the yaw of the robot (reset field oriented drive)
     */
    public void resetYaw() {
        setYaw(0);
    }


}
