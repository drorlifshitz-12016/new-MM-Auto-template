package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.MMRobot;

public class RollerIntake extends SubsystemBase{

    MMRobot mmRobot = MMRobot.getInstance();

    CRServo servo;
    public boolean isRoll = false;
    public RollerIntake() {
        servo = MMRobot.getInstance().mmSystems.hardwareMap.crservo.get("intakeRoller");

    }

    public void setPower(double power){servo.setPower(power);}


}
