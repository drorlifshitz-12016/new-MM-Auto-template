package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleServo;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.Utils.Configuration;

public class IntakeArm extends SubsystemBase {

    MMRobot mmRobot = MMRobot.getInstance();

    CuttleServo servoRight;
    CuttleServo servoLeft;

    public enum Position {
        IN(0),
        OUT(1);
        public double intakeArmPosition;
        Position(double position) {
            this.intakeArmPosition = position;
        }
    }


    public IntakeArm() {
        servoRight = new CuttleServo(mmRobot.mmSystems.controlHub, Configuration.ARM_ANGLE_RIGHT);
        servoLeft = new CuttleServo(mmRobot.mmSystems.controlHub, Configuration.ARM_ANGEL_LEFT);
    }

    public void setPosition(double position) {
        servoLeft.setPosition(position);
        servoRight.setPosition(1-position);
    }

    public double getPosition() {
        return servoRight.getPosition();
    }

}
