//package org.firstinspires.ftc.teamcode;
//
//import com.arcrobotics.ftclib.command.CommandBase;
//import com.arcrobotics.ftclib.command.CommandOpMode;
//import com.arcrobotics.ftclib.gamepad.GamepadEx;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.arcrobotics.ftclib.command.CommandOpMode;
//import com.arcrobotics.ftclib.command.CommandScheduler;
//import com.arcrobotics.ftclib.command.ConditionalCommand;
//import com.arcrobotics.ftclib.command.InstantCommand;
//import com.arcrobotics.ftclib.command.SequentialCommandGroup;
//import com.arcrobotics.ftclib.command.WaitCommand;
//import com.arcrobotics.ftclib.gamepad.GamepadEx;
//import com.arcrobotics.ftclib.gamepad.GamepadKeys;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.Subsystems.Arm;
//import org.firstinspires.ftc.teamcode.Commands.ArmRotCommand;
//import com.arcrobotics.ftclib.command.CommandScheduler;
//
//
//public class CommandTeleV1 extends CommandOpMode {
//
//    private Arm arm_sub;
//
//    private ArmRotCommand ArmRotCommand;
//
//    int target_pos = 50;
//
//    private GamepadEx Driver;
//    private GamepadEx Gunner;
//
//
//    @Override
//    public void initialize() {
//        CommandScheduler.getInstance().reset();
//        arm_sub = new Arm(hardwareMap);
//
//
//        ArmRotCommand = new ArmRotCommand(arm_sub,target_pos);
//
//
//    }
//
//    @Override
//    public void run(){
//        CommandScheduler.getInstance();
//
//        Gunner.getGamepadButton(GamepadKeys.Button.B)
//                .whenPressed(new ArmRotCommand(arm_sub,target_pos));
//
//
//
//
//
//
//    }
//}
