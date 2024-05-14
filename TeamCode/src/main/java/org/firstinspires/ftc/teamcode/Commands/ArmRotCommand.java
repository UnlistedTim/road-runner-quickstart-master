//package org.firstinspires.ftc.teamcode.Commands;
//
//import com.arcrobotics.ftclib.command.CommandBase;
//import com.arcrobotics.ftclib.command.Subsystem;
//import com.arcrobotics.ftclib.controller.PIDController;
//import com.arcrobotics.ftclib.controller.PIDFController;
//
//import org.firstinspires.ftc.teamcode.Subsystems.Arm;
//
//public class ArmRotCommand extends CommandBase {
//    private Arm arm_sub;
//
//    private PIDController Arm_controller;
//    public static double Arm_Position = 0;
//    public static double Arm_Velocity = 0;
//    public static double controllerOutput = 0;
//    private double target_position;
//
//    private double tolerance = 3;
//
//
//
//
//    public ArmRotCommand(Arm subs, int target_pos){
//        arm_sub = subs;
//        addRequirements(arm_sub);
//
//        this.target_position = target_pos;
//
//        Arm_controller = new PIDController(Arm.p,Arm.i,Arm.d);
//
//
//
//    }
//
//    @Override
//    public void initialize() {
//        Arm_controller.reset();
//        Arm_controller.setSetPoint(target_position);
//
//    }
//
//    @Override
//    public void execute(){
//        Arm_Position = arm_sub.get_arm_pos();
//        Arm_Velocity = arm_sub.get_arm_vel();
//
//        Arm_controller.setSetPoint(target_position);
//
//        controllerOutput = Arm_controller.calculate(Arm_Position,target_position);
//        double ff = Math.cos(Math.toRadians(Arm_Position/Arm.ticks_in_degree)) * Arm.f;
//        controllerOutput+=ff;
//
//        //Update the lift power with the controller
//
//        arm_sub.set_arm_power(controllerOutput);
//    }
//
//    @Override
//    public boolean isFinished() {
//        return Math.abs(target_position - Arm_Position) <= tolerance;
//    }
//
//    @Override
//    public void end(boolean interrupted){
//        double ff = Math.cos(Math.toRadians(Arm_Position/Arm.ticks_in_degree)) * Arm.f;
//        arm_sub.set_arm_power(ff);
//    }
//
//}
