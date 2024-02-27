package org.firstinspires.ftc.teamcode.SubSystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Utils.Configuration;


@Config
public class Turret extends SubsystemBase {
    private final CRServo turretServoA;
    private final CRServo turretServoB;
    private final DcMotor turretEncoder;
    private final double TICKS_PER_REV = 8192;
    private final double GEAR_RATIO = (21./95);
    public static double kP = 0.045;
    public static double kI = 0;
    public static double kD = 0;

    private final PIDController pidController = new PIDController(kP, kI, kD);

    public Turret(HardwareMap hardwareMap) {
        turretServoA = hardwareMap.crservo.get(Configuration.TURRET_RIGHT);
        turretServoB = hardwareMap.crservo.get(Configuration.TURRET_LEFT);
        turretServoA.setDirection(DcMotorSimple.Direction.REVERSE);
        turretServoB.setDirection(DcMotorSimple.Direction.REVERSE);
        turretEncoder = hardwareMap.dcMotor.get(Configuration.TURRET_ENCODER);
        turretEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //resets encoder to 0
        turretEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //makes it use power from -1 to 1

    }
    public void setPower (double power) {
        power = Math.min(power,1);
        power = Math.max(power,-1);
        turretServoA.setPower(power);
        turretServoB.setPower(power);
    }
    public double getAngle(){
        return turretEncoder.getCurrentPosition()/TICKS_PER_REV * 360 * GEAR_RATIO;
    }

    public void stop(){
        setPower(0);
    }

    public PIDController getPidController() {
        return pidController;
    }

    public void telemetry() {
        FtcDashboard.getInstance().getTelemetry().addData("Turret Angle", getAngle());
        FtcDashboard.getInstance().getTelemetry().addData("Turret Target Angle", getPidController().getSetPoint());
        FtcDashboard.getInstance().getTelemetry().addData("Calculated Turret Power", getPidController().calculate(getAngle()));
        FtcDashboard.getInstance().getTelemetry().update();
    }

    @Override
    public void periodic() {
        telemetry();
    }
}

