//package org.firstinspires.ftc.teamcode;
//
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.arcrobotics.ftclib.controller.PIDController;
//import com.arcrobotics.ftclib.controller.wpilibcontroller.ArmFeedforward;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.arcrobotics.ftclib.hardware.motors.MotorEx;
//import com.arcrobotics.ftclib.controller.PIDFController;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//@Config
//@TeleOp
//
//public class PIDF_Arm extends OpMode {
//    private PIDController controller;
//    private ArmFeedforward controll1;
//
//
//
//
//    // Fully Extension constants (This is for the fully extended arm, More force is required)
//    // p = 0.05, i = 0, f = 0.2, d = 0.002
//
//    //Non extended constants
//    // p = 0.001, f = 0.15, i = 0, d = 0.0005
//
//    public static final double p = 0, i = 0, d = 0;
//    public static final double f = 0;
//
//    public static int target = 0;
//
//    public static final double ticks_in_degree = 2786.2/360.0;
//
//    private DcMotorEx arm_rotate;
//    private DcMotorEx arm_slide;
//    @Override
//    public void init(){
//        controller = new PIDController(p,i,d);
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        arm_rotate = hardwareMap.get(DcMotorEx.class,"arm_rotate");
//        arm_slide = hardwareMap.get(DcMotorEx.class, "arm_slide");
//        arm_rotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        arm_rotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        arm_slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        arm_slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        arm_slide.setTargetPosition(0);
//        arm_slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//
//
//
//
//    }
//
//    @Override
//    public void loop(){
//        arm_slide.setTargetPosition(2000);
//        arm_slide.setVelocity(1000);
//        controller.setPID(p,i,d);
//
//
//        int armPos = arm_rotate.getCurrentPosition();
//        double pid = controller.calculate(armPos,target);
//        double ff = Math.cos(Math.toRadians(armPos/ticks_in_degree)) * f;
//
//        double power = pid + ff;
//
//        arm_rotate.setPower(power);
//
//        telemetry.addData("pos ", armPos);
//        telemetry.addData("target ", target);
//        telemetry.update();
//
//
//
//
//
//    }
//
//}
