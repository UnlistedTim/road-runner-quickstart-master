//package org.firstinspires.ftc.teamcode.Subsystems;
//
//import com.arcrobotics.ftclib.command.CommandScheduler;
//import com.arcrobotics.ftclib.command.SubsystemBase;
//import com.arcrobotics.ftclib.controller.PIDController;
//import com.arcrobotics.ftclib.controller.PIDFController;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//public class Arm extends SubsystemBase {
//
//    private DcMotorEx arm_rotate;
//    private DcMotorEx arm_slide;
//
//    private PIDController controller;
//
//
//
//    /**
//     * Creates a new ExampleSubsystem.
//     */
//    public Arm(final HardwareMap hardwareMap) {
//        arm_rotate = hardwareMap.get(DcMotorEx.class, "arm_rotate");
//
//    }
//
//    public final static int DOWN = 50;
//
//    public final static int UP = 900;
//
//    public static final double ticks_in_degree = 2786.2/360.0;
//
//    public static double p = 0.05, i = 0, d = 0.002;
//    public static double f = 0.2;
//
//
//
//
//
//    public int get_arm_pos(){
//        return arm_rotate.getCurrentPosition();
//
//    }
//
//    public double get_arm_vel(){
//        return arm_rotate.getVelocity();
//    }
//
//    public void set_arm_power(double power){
//        arm_rotate.setPower(power);
//    }
//
//    public void reset_arm_pos(){
//        arm_rotate.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        arm_rotate.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//    }
//
//    public void go_to_pos(int current,int target){
//        controller.setPID(p,i,d);
//        double pid = controller.calculate(current,target);
//        double ff = Math.cos(Math.toRadians(current/ticks_in_degree)) * f;
//        double power = pid + ff;
//
//        arm_rotate.setPower(power);
//    }
//
//    @Override
//    public void periodic() {
//
//        CommandScheduler.getInstance().run();
//
//        // This method will be called once per scheduler run
//
//    }
//}
