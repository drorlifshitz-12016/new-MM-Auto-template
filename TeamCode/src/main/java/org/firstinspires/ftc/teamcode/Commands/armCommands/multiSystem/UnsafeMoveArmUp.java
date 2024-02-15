package org.firstinspires.ftc.teamcode.Commands.armCommands.multiSystem;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.ArmPosition;
import org.firstinspires.ftc.teamcode.Commands.armCommands.antiTurret.AntiTurretGetToPosition;
import org.firstinspires.ftc.teamcode.Commands.armCommands.elbow.ElbowGetToPosition;
import org.firstinspires.ftc.teamcode.Commands.armCommands.elevator.ElevatorGetToHeightPID;
import org.firstinspires.ftc.teamcode.Commands.armCommands.extender.ExtenderSetPosition;
import org.firstinspires.ftc.teamcode.Commands.armCommands.turret.RotateTurretByPID;
import org.firstinspires.ftc.teamcode.SubSystems.AntiTurret;
import org.firstinspires.ftc.teamcode.SubSystems.Elbow;
import org.firstinspires.ftc.teamcode.SubSystems.Elevator;
import org.firstinspires.ftc.teamcode.SubSystems.Extender;
import org.firstinspires.ftc.teamcode.SubSystems.Turret;

public class UnsafeMoveArmUp extends SequentialCommandGroup {
    public UnsafeMoveArmUp(Elevator elevator, Elbow elbow, Extender extender, Turret turret, AntiTurret antiTurret, ArmPosition position, boolean isLeftOfBoard) {
        super(
                new ElevatorGetToHeightPID(elevator, position.getElevatorHeight()),
                new RotateTurretByPID(turret, position.getTurretAngle(isLeftOfBoard)),
                new ElbowGetToPosition(elbow, position.getElbowPosition()), /*These are instant commands so their isFinished always true */
                new WaitCommand(UnsafeMoveArm.EXTENDER_WAIT_TIME), //Trying to avoid elbow's servos overload
                new ExtenderSetPosition(extender, position.getExtenderPosition()),
                new AntiTurretGetToPosition(antiTurret, position.getAntiTurretPosition())
        );
    }
}