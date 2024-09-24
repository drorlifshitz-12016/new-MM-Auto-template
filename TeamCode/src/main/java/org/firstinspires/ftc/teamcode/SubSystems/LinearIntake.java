package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleServo;
import org.firstinspires.ftc.teamcode.MMRobot;

public class LinearIntake extends SubsystemBase {

    CuttleServo rightServo;
    CuttleServo leftServo;

    final double OPEN = 0;
    final double CLOSE = 0.25;

    public LinearIntake() {
        rightServo = new CuttleServo(MMRobot.getInstance().mmSystems.controlHub, 1);
        leftServo = new CuttleServo(MMRobot.getInstance().mmSystems.controlHub, 2);
    }
    public void setPosition(double position) {
        rightServo.setPosition(1-position);
        leftServo.setPosition(position);
    }

    public double getPosition() {
        return rightServo.getPosition();
    }



}
