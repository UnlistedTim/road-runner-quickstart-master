package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import android.util.Size;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.List;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;
import   java.lang.Math;
import java.util.concurrent.TimeUnit;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


public class BaseClass extends MecanumDrive {
//    protected DcMotorEx leftFront;
//    protected DcMotorEx rightFront;
//    protected DcMotorEx leftBack;
//    protected DcMotorEx rightBack;
    protected DcMotorEx arm_rotate;
    protected DcMotorEx arm_slide;
    //    protected Servo arm_grab;
    protected Servo arm_handle;
    protected Servo drone;
    WebcamName Webcam1;

    public static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    public int DESIRED_TAG_ID = 5;     // Choose the tag you want to approach or set to -1 for ANY tag.
     VisionPortal visionPortal;               // Used to manage the video source.
     AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
     AprilTagDetection desiredTag = null;

    protected Servo top_claw;
    protected Servo bottom_claw;
    protected DistanceSensor left_dist;
    protected DistanceSensor front_dist;


    public ElapsedTime runtime = new ElapsedTime();
    public LinearOpMode Op;
    double st, st1, st2, st3, st4, st5, st6, strb;
    double robotVoltage;
    public static boolean right = false;

    public static boolean baseblue = false, baseright = true;
    public static double base_align_angle = 90; // change later
    public static int base_apr_id;

    double power;

    //Servo preset value
    double arm_handle_ip0 = 0.54, arm_handle_ip1 = 0.585, arm_handle_ip2, arm_handle_ip3, arm_handle_ip4, arm_handle_ip5, arm_handle_idle = 0.44;
    double arm_handle_op1 = 0.05, arm_handle_op2 = 0.05, arm_handle_op3 = 0.05, arm_handle_op4 = 0.05, arm_handle_op6, arm_handle_op7 = 0.03, arm_handle_op8, arm_handle_op9;
    double arm_grab_hold = 0.34, arm_grab_idle = 0.0, arm_grab_open1, arm_grab_open2 = 0.2;
    double top_claw_hold = 0.73, top_claw_idle = 0.8, top_claw_open = 0.9;
    double bottom_claw_hold = 0.3, bottom_claw_idle = 0.2, bottom_claw_open = 0.1;

    double front_dist_low = 100, front_dist_high = 300;
    double left_dist_low = 100, left_dist_high = 300;
    double handle_dist_det = 0.33;

    //Motor preset value

    int arm_rotate_ground = 0, arm_rotate_buffer = 300, arm_rotate_intake_pre = 125, arm_rotate_ip1, arm_rotate_ip2, arm_rotate_ip3, arm_rotate_ip4, arm_rotate_ip5;
    int arm_rotate_op1 = 1600, arm_rotate_op2 = 1600, arm_rotate_op3 = 1600, arm_rotate_op4 = 1600, arm_rotate_op5, arm_rotate_op6, arm_rotate_op7 = 1600, arm_rotate_op8, arm_rotate_op9, arm_rotate_out_buffer = arm_rotate_op4 - 150;
    //arm rotate new 1500
    int arm_rotate_hang = 900;
    int arm_slide_extend = 0, arm_slide_turn = -3000, arm_slide_idle = -5000, arm_slide_collapse = -5200, arm_slide_hang = -4000, arm_slide_auto = -530, arm_slide_intake = -4000;
    int arm_slide_op1 = -4800, arm_slide_op2 = -3800, arm_slide_op3 = -3000, arm_slide_op4 = -2200, arm_slide_op5 = -1400, arm_slide_op6 = -400, arm_slide_op7, arm_slide_op8, arm_slide_op9;

    //1: -4800 2: -4000 3: -3200; 4: -2400
    //+ offset = -4800, -3800, -3000, -2200
    //int arm_slide_hang = -600;


    final double SPEED_GAIN = 0.0005; // 0.02  //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double AUTOSPEED_GAIN = 0.01;
    final double STRAFE_GAIN = 0.07; //0.03  //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN = 0.01;  //0.015  //  Turn Controtl "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.36;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE = 0.4;   //  Clip he approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)
    double DESIRED_DISTANCE_T = 15;//INCH
    //double DESIRED_DISTANCE_A =10;//INCH?

    double rangeError = 20;
    int finalcycle = 3;
    double headingError = 10;
    double yawError = 10;
    double STRAFE_TARG = 3;
    double drive = 0;        // Desired forward power/speed (-1 to +1)
    double strafe = 0;        // Desired strafe power/speed (-1 to +1)
    double turn = 0;        // Desired turning power/speed (-1 to +1)


    int outtake_1_delay = 0;
    int outtake_2_delay = 0;
    // Pose2d P;

    final double HSPEED_GAIN = 0.025; // 0.02  //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    double HSTRAFE_GAIN = 0.018; //0.01,0.02  //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double HTURN_GAIN = 0.032;  //0.015  //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double HMAX_AUTO_SPEED = 0.3;   //  Clip the approach speed to this max value (adjust for your robot)
    final double HMAX_AUTO_STRAFE = 0.28; //0.5  //  Clip the approach speed to this max value (adjust for your robot)
    final double HMAX_AUTO_TURN = 0.25;   //  Clip the turn speed to this max value (adjust for your robot)

    double april_left_strafe = -6.6, april_right_strafe = 8.5;  //8.0
    double HrangeError = 20, walltarget = 3, temp1, temp2, temp3;
    double HheadingError = 10;
    double HyawError = 10;
    double Hdrive = 0;        // Desired forward power/speed (-1 to +1)
    double Hstrafe = 0;        // Desired strafe power/speed (-1 to +1)
    double Hturn = 0;        // Desired turning power/speed (-1 to +1)
    Pose2d startPoseA = new Pose2d(0, 0, 0);

    double gap = 100;
    long[] delay = {0, 0, 0, 0};
    final double[] row1x = {0.42, 0.175, 0.295, 0.365, 0.47, 0.555, 0.675};
    final double[] row2x = {0.42, 0.105, 0.24, 0.33, 0.42, 0.51, 0.61, 0.766};

    final int[] row1y = {310, 440, 350, 340, 340, 350, 440};
    final int[] row2y = {540, 750, 610, 580, 560, 570, 620, 750};

    //    final  double[] row1xc = {0.42, 0.42,0.42, 0.42, 0.185, 0.285, 0.375};
    final double[] row1xc = {0.42, 0.42, 0.42, 0.42, 0.205, 0.305, 0.395};
    final double[] row2xc = {0.42, 0.42, 0.42, 0.42, 0.1, 0.245, 0.335, 0.42};

    final int[] row1yc = {350, 350, 350, 350, 420, 360, 350};
    final int[] row2yc = {550, 550, 550, 550, 740, 610, 560, 550};

    boolean roll_flag = false, timeend = false;


    double finalx = 0.42;
    int finaly = 750;

    final int slide_vel = 3500;
    final int arm_vel = 1600;


    int input1x = 0, currentx = 4;
    int input1y = 0, currenty = 2;
    int tempinput = 1, intake_done_step = 0;
    int index = 2;
    double husk_tar = 110;//110
    static int encoder;


    int input2x = 0;
    int input2y = 0;

    class Cservo {
        Servo name;

        double[] postion;

        Cservo(Servo s, double v1, double v2) {
            this.name = s;
            name.scaleRange(v1, v2); // limit operation interval of the servo
            name.setDirection(Servo.Direction.REVERSE); // Reverse the Servo rotating direction
            double name_open = 0.3, name_close = 0.5, name_hold = 0.3; //define preset positions
            name.setPosition(name_open);
            name.setPosition(name_close);
            name.setPosition(name_hold);

        }


    }


    int tmp1 = 1000, tmp2 = 1000;


    int[] out_height = {0, 350, 425, 655, 885, 1115, 1345, 1450, 1750, 1800, 140};// 0, withdraw, l,m,h // 700 old
    final double RPS = 2787 * 0.9;

    //  int height_level = 2;//Default postion level
    //int move_level = 3; // Default positon level
    double[] out_move = {0.12, 0.27, 0.35, 0.42, 0.49, 0.57, 0.72};//
    //   double[] fine_move = {-0.04, -0.02, 0, 0.02, 0.04};


    double[] in_handle = {0.165, 0.975, 0.94, 0.43, 0.6, 0.860, .98}; // start_fold,grab, move, move_fold,,hinge,auto,auto_left;0.962
    // double[] in_handle = {0.14,0.975,0.94,0.3,0.6,0.86}; //close/fold, intake, move,hinge,auto,auto_left;0.962

    double[] out_handle = {0.16, 0.34, 0.48, 0.62, 0.78, 0.12, 1.0};

    double[] out_cam = {0, 0.17, 0.34};
    //handle 0.375, arm 0.001 , grab 0.30


    int drop_qty = 2;
//    int bred = 0, bgreen = 0, bblue = 0, tred = 0, tgreen = 0, tblue = 0;


    boolean intake = true;

    boolean lifting = false, intake_rolling = false, outtaking = false, driving = true, bot_fla = false, top_fla = false;
    boolean forcing = false, leveling = false, launching = false, turning = false, out_handling = false;
    boolean vri1 = false, vir2 = false, roll_backing = false, forwarding = false, outtake_dropping = false;
    boolean aligning = false, adjusting = false, printing = true, init_setup = true, april_ready = false, setup_mode = false;
    boolean intake_grabing = false, pix_full = false, dropping = false, cor_updating1 = false, cor_updating2 = false, adj_key_reset = true, fadj_key_reset = true, grab_folding = true;
    int grab_step = 0, align_step = 0, servo_step = 0, post_step = 0, drop_step = 0, done_step = 0, color_step = 0, turn_step = 0, input_count = 0, roll_back_step = 0, intake_step = 0;
    boolean touching = false, out_handle_in = false;
    boolean half_full = false, wall_route = false, side_align = false;
    double headingang, targetang, turnang, stinput = 0, endgame, fine_move_offset = 0, time_period = 0, stconfirm = 0;

    double drone_idle = 0.85, drone_launch = 1.0;
    int folding_step = 0, grab_counter = 0;
    boolean folding = false, intake_ready = false, prev_side_align = false;
    double ax11, ax12, ax21, ax22;
    double delta_dis = 0;
    int cycle = 0;// pile intake

    int layer_rate = 230;
    int drop_offset = 310;//300
    int x = 0;
    double ax, ay;
    boolean volthi = false, acal = false;
    double y;


    //private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    //private static final int DESIRED_TAG_ID = 1; //-1    // Choose the tag you want to approach or set to -1 for ANY tag.

    boolean targetFound = false;    // Set to true when an AprilTag target is detected
    IMU imu;
    IMU.Parameters imuparameters = new IMU.Parameters(new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
            RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));


    public BaseClass(LinearOpMode linearOpMode, Pose2d pose) {
        super(linearOpMode.hardwareMap, pose);
        Op = linearOpMode;
     initHardware(Op.hardwareMap);
    }

    public BaseClass(HardwareMap hardwareMap, Pose2d pose1) {
        super(hardwareMap, pose1);
     initHardware(hardwareMap);
    }


    public void husk_alignb() {
// 240 120
//220 110
//200 108
//190 105
//180  99

        int READ_PERIOD = 1;
        int j = 0;
        double y0 = 240;
        int ytarget = 200;
        x = 0;
        y = 0;
        // intake_handle(2);
        if (volthi) HSTRAFE_GAIN = 0.016;
        //timer3(0);
        Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);
        rateLimit.expire();

        timer3(0);
        if (y > 2000) y = y0;


        while (!timer3(1000)) {


            //   husk[j]=x;
//            if(j<24)j++;
            if (y > 1000) y = y0;
            y0 = y;
            HrangeError = (y - ytarget) / 10;
            husk_tar = (y - 180) * 22 / 60 + 100;
            //   husk_tar=(y-180)/3+98;

            if (x > 0) HyawError = (x - husk_tar);// 110 iss the target x valule for alignment
            else HyawError = 0;
            HheadingError = base_align_angle - imu.getRobotYawPitchRollAngles().getYaw((AngleUnit.DEGREES));
            Hdrive = Range.clip(HrangeError * HSPEED_GAIN, -HMAX_AUTO_SPEED, HMAX_AUTO_SPEED);
            Hstrafe = Range.clip(-HyawError * HSTRAFE_GAIN, -HMAX_AUTO_STRAFE, HMAX_AUTO_STRAFE);
            Hturn = Range.clip(HheadingError * HTURN_GAIN, -HMAX_AUTO_TURN, HMAX_AUTO_TURN);
            moveRobot(Hdrive, Hstrafe, Hturn, 1,true);
            if (x > 0 && Math.abs(HyawError) < 12) break;
        }
        stop_drive();
    }


    public void finalxy() {

        if (drop_qty == 2) {
            if (!adjusting) {
                if (input1x > 0) {
                    currentx = input1x;
                }
                if (input1y > 0) {
                    currenty = input1y;
                }


            }
            cor_updating1 = false;
        } else {
            if (!adjusting) {
                if (input2x > 0) {
                    currentx = input2x;
                }
                if (input2y > 0) {
                    currenty = input2y;
                }


            }
            cor_updating2 = false;
        }


        // if(currentx==0||currenty==0) vir2=true;
//        if(currenty==0 || currentx==0) return;
        if (side_align) {
            if (currentx < 4) {
                vir2 = true;
                return;
            }
            if (currenty % 2 == 0) {

                finalx = row2xc[currentx];
                finaly = row2yc[currentx] + (currenty - 2) * layer_rate;
                finaly += drop_offset;
            } else {
                if (currentx == 7) {// may change to 6;
                    currentx = 6;
                    vir2 = true;

                }
                finalx = row1xc[currentx];
                finaly = row1yc[currentx] + (currenty - 1) * layer_rate;
                finaly += drop_offset;
                //if(currenty==1) finaly-=30;
            }
        } else {
            if (currenty % 2 == 0) {

                finalx = row2x[currentx];
                finaly = row2y[currentx] + (currenty - 2) * layer_rate;
                finaly += drop_offset;
            } else {
                if (currentx == 7) {// may change to 6;
                    currentx = 6;
                    vir2 = true;

                }
                finalx = row1x[currentx];
                finaly = row1y[currentx] + (currenty - 1) * layer_rate;
                finaly += drop_offset;

                //if(currenty==1) finaly-=30;
            }


        }
        if (currenty == 1) finaly = finaly - 35;
        prev_side_align = side_align;

    }

    public void ZeroSlide() {
        arm_slide.setTargetPosition(0);
        arm_slide.setVelocity(1500);
    }

    public void strafe(double power) {
        leftFront.setPower(power);
        rightFront.setPower(-power);
        leftBack.setPower(-power);
        rightBack.setPower(power);
    }


    public void initHardware(HardwareMap hardwareMap) {
//        imu = hardwareMap.get(IMU.class, "imu");
//        imu.initialize(imuparameters);
//        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
//        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
//        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
//        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

        arm_rotate = hardwareMap.get(DcMotorEx.class, "arm_rotate");
        arm_slide = hardwareMap.get(DcMotorEx.class, "arm_slide");
        imu = hardwareMap.get(IMU.class, "imu");

//        arm_grab = hardwareMap.get(Servo.class, "arm_grab");
        top_claw = hardwareMap.get(Servo.class, "top_claw");
        bottom_claw = hardwareMap.get(Servo.class, "bottom_claw");
        arm_handle = hardwareMap.get(Servo.class, "arm_handle");
        drone = hardwareMap.get(Servo.class, "drone");

        left_dist = hardwareMap.get(DistanceSensor.class, "left_dist");
        front_dist = hardwareMap.get(DistanceSensor.class, "front_dist");
        Webcam1=hardwareMap.get(WebcamName.class, "Webcam 1");


        //arm_grab.scaleRange(0.44,0.84);
//        arm_handle.scaleRange(0.07, 0.77);


        arm_rotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm_rotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm_rotate.setTargetPosition(0);
        arm_rotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        arm_slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm_slide.setTargetPosition(0);
        arm_slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        arm_slide.setVelocity(0);

        arm_handle.setDirection(Servo.Direction.REVERSE);
        drone.setDirection(Servo.Direction.REVERSE);

//        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
//        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
//
//
//
//
//        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        IMU.Parameters imuparameters = new IMU.Parameters(new RevHubOrientationOnRobot(
//                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
//                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
//
//        imu = hardwareMap.get(IMU.class, "imu");
//
//        imu.initialize(imuparameters);
//        imu.resetYaw();


        initAprilTag();
        if (USE_WEBCAM)  setManualExposure(6, 150);  // Use low exposure time to reduce motion blur

    }


    public void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder()
                .setLensIntrinsics(822.317, 822.317, 319.495, 242.502)
                .build();



        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
            visionPortal = new VisionPortal.Builder()
                   .setCamera(Webcam1)
                    .setCameraResolution(new Size(640, 480))
                    .addProcessor(aprilTag)
                    .build();
    }

//    public boolean april_align( int id) {
//        double dis = 0;
//        double april_strafe;
////        double s_gain = SPEED_GAIN;
//
//
//        targetFound = false;
//        desiredTag = null;
//
//        if (side_align){
//            id = id+2;
//            april_strafe = april_right_strafe;
//        }
//        else april_strafe = april_left_strafe;
//
//
//
//        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
//        for (AprilTagDetection detection : currentDetections) {
//
//            // Look to see if we have size info on this tag.
//            if (detection.metadata != null) {
//
//                //  Check to see if we want to track towards this tag.
//                if ((detection.id == id)) {//(DESIRED_TAG_ID < 0) ||
//                    // Yes, we want to use this tag.
//                    desiredTag = detection;
//                    //  if(acal) {ax=desiredTag.ftcPose.x;ay=out_right_dis.getDistance(DistanceUnit.INCH)-10;acal=false;return true;}
//                    targetFound = true;
//                    break;  // don't look any further.
//                }
//
//            }
//        }
//
//
//        if (targetFound) {
//            dis = out_left_dis.getDistance(DistanceUnit.CM);
//
//            if (dis > 1000) dis = 400;
//            rangeError = dis - DESIRED_DISTANCE_T;
//            headingError = base_align_angle - imu.getRobotYawPitchRollAngles().getYaw((AngleUnit.DEGREES));
//            yawError = desiredTag.ftcPose.bearing + april_strafe;
////                yawError = desiredTag.ftcPose.x - 2.0; //-6.6
//            drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
//
//            strafe = Range.clip(yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
//            turn = -Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
//
//
//            moveRobot(drive, strafe, turn,-1);//outake =-1, intake=1;
//
//        }
//        else {
//            driving=true;
//            return false;
//
//        }
//
//
//
//
//        return false;
//    }


    protected void field_centric(double iy, double ix, double irx) {
        double y = iy; // Remember, this is reversed!
        double x = ix * 1.1; // Counteract imperfect strafing
        double rx = irx * 0.75;

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

//        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
//        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);


        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        leftFront.setPower(frontLeftPower);
        leftBack.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightBack.setPower(backRightPower);

    }


    public void straft(boolean right, double power) {

        double dir = 1;
        if (right) dir = -1;
        // double  power=1;
        leftFront.setPower(dir * (power + 0.035));
        leftBack.setPower(-dir * power);
        rightFront.setPower(-dir * (power + 0.03));
        rightBack.setPower(dir * power);
    }


    protected void robot_centric(double iy, double ix, double irx, double ratio) {

        double y = -iy;
        double x = ix * 1.1; // Counteract imperfect strafing
        double rx = irx * 0.75;
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        leftFront.setPower(frontLeftPower * ratio);
        rightFront.setPower(frontRightPower * ratio);
        leftBack.setPower(backLeftPower * ratio);
        rightBack.setPower(backRightPower * ratio);


    }

    protected void vel_robot_centric(double iy, double ix, double irx, double ratio) {

        double y = -iy;
        double x = ix; // Counteract imperfect strafing 1.1
        double rx = irx ; // 0.75
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;



        motor_vel(leftFront,frontLeftPower * ratio,true);
        motor_vel(rightFront, frontRightPower * ratio,true);
        motor_vel(leftBack,backLeftPower * ratio,true);
        motor_vel(rightBack,backRightPower * ratio,true);


    }

    protected void demo_robot_centric(double iy, double ix, double irx) {
        double y = iy;
        double x = -ix * 1.1; // Counteract imperfect strafing
        double rx = -irx * 0.75;
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        leftFront.setPower(-frontLeftPower * 0.3);
        rightFront.setPower(-frontRightPower * 0.3);
        leftBack.setPower(-backLeftPower * 0.3);
        rightBack.setPower(-backRightPower * 0.3);


    }


    void function1(double speed, int sleep) {


    }


    public void forward(double power, long ms) {


        leftFront.setPower(power);
        leftBack.setPower(power);
        rightFront.setPower(power);
        rightBack.setPower(power);

        pause(ms);
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);

    }


    private boolean timer(double period) {

        if (period == 0) {
            st = runtime.milliseconds();
            return false;
        }

        if (runtime.milliseconds() - st > period) return true;

        return false;

    }

    private boolean timer1(double period) {

        if (period == 0) {
            st1 = runtime.milliseconds();
            return false;
        }

        if (runtime.milliseconds() - st1 > period) return true;

        return false;

    }

    boolean timer2(double period) {

        if (period == 0) {
            st2 = runtime.milliseconds();
            return false;
        }

        if (runtime.milliseconds() - st2 > period) return true;

        return false;

    }

    public void fl_vel(double percent){
        if (percent > 1.0 || percent < -1.0) return;
        if (Math.abs(percent *RPS) < 50){
            leftFront.setPower(0);
            return;
        }
        leftFront.setVelocity(percent * RPS);
    }

    public void fr_vel(double percent){
        if (percent > 1.0 || percent < -1.0) return;
        if (Math.abs(percent *RPS) < 50){
            rightFront.setPower(0);
            return;
        }
        rightFront.setVelocity(percent * RPS);
    }

    public void rl_vel(double percent){
        if (percent > 1.0 || percent < -1.0) return;
        if (Math.abs(percent *RPS) < 50){
            leftBack.setPower(0);
            return;
        }
        leftBack.setVelocity(percent * RPS);
    }
    public void rr_vel(double percent){
        if (percent > 1.0 || percent < -1.0) return;
        if (Math.abs(percent *RPS) < 50){
            rightBack.setPower(0);
            return;
        }
        rightBack.setVelocity(percent * RPS);
    }
    public void motor_vel(DcMotorEx mot,double percent, boolean ignore){
        //if (percent > 1.0 || percent < -1.0) return;
        if (ignore && Math.abs(percent) <0.05) percent=0;
        mot.setVelocity(percent * RPS);
    }

    boolean timer4(double period) {

        if (period == 0) {
            st4 = runtime.milliseconds();
            return false;
        }

        if (runtime.milliseconds() - st4 > period) return true;

        return false;

    }

    boolean timerroollback(double period) {

        if (period == 0) {
            strb = runtime.milliseconds();
            return false;
        }

        if (runtime.milliseconds() - strb > period) return true;

        return false;

    }


    boolean timer5(double period) {

        if (period == 0) {
            st5 = runtime.milliseconds();
            return false;
        }

        if (runtime.milliseconds() - st5 > period) return true;

        return false;

    }


    boolean timer3(double period) {

        if (period == 0) {
            st3 = runtime.milliseconds();
            return false;
        }

        if (runtime.milliseconds() - st3 > period) return true;

        return false;

    }

    boolean timerinput(double period) {

        if (period == 0) {
            stinput = runtime.milliseconds();
            return false;
        }

        if (runtime.milliseconds() - stinput > period) return true;

        return false;

    }

    boolean timerconfirm(double period) {

        if (period == 0) {
            stconfirm = runtime.milliseconds();
            return false;
        }

        if (runtime.milliseconds() - stconfirm > period) return true;

        return false;

    }

    public void move(double power) {
        leftFront.setPower(power);
        rightFront.setPower(power);
        leftBack.setPower(power);
        rightBack.setPower(power);
    }

    public void stop_drive() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }


//


    public final void pause(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }


    public void  moveRobot(double x, double y, double yaw, int intake, boolean auton) {
        // Calculate wheel powers.
        double leftFrontPower = x - y - yaw;
        double rightFrontPower = x + y + yaw;
        double leftBackPower = x + y - yaw;
        double rightBackPower = x - y + yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }




        // Send powers to the wheels.
        leftFront.setPower(leftFrontPower * intake);
        leftBack.setPower(leftBackPower * intake);
        rightFront.setPower(rightFrontPower * intake);
        rightBack.setPower(rightBackPower * intake);

    }


    public boolean setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

//        if (visionPortal == null) {
//            return false;
//        }
        // pause(3000);
        // Make sure camera is streaming before we try to set the exposure controls
//        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
////            telemetry.addData("Camera", "Waiting");
////            telemetry.addData("Current state",visionPortal.getCameraState());
////            telemetry.update();
        if ((visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
            pause(20);
        } else {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                pause(50);
            }
            exposureControl.setExposure((long) exposureMS, TimeUnit.MILLISECONDS);
            pause(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            pause(20);
            april_ready = true;
            return true;
        }
//            telemetry.addData("Camera", "Ready");
//            telemetry.update();


        // Set camera controls unless we are stopping.
//        if (!isStopRequested())

        return false;
    }


//    public boolean iniAprilTag(WebcamName cam) {
//
//        if (init_setup) {
//            aprilTag = new AprilTagProcessor.Builder()
//                    .setLensIntrinsics(822.317, 822.317, 319.495, 242.502)
//                    .build();
////"focalLength="822.317f, 822.317f"principalPoint="319.495f, 242.502f
//            // Adjust Image Decimation to trade-off detection-range for detection-rate.
//            // eg: Some typical detection data using a Logitech C920 WebCam
//            // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
//            // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
//            // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
//            // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
//            // Note: Decimation can be changed on-the-fly to adapt during a match.
//            aprilTag.setDecimation(3);
//
//            // Create the vision portal by using a builder.
////            visionPortal = new VisionPortal.Builder()
////                    .setCamera(cam)
////                    .setCameraResolution(new Size(640, 480))
////                    .addProcessor(aprilTag)
////                    .build();
//            init_setup = false;
//        }
//        // return setManualExposure(6, 250);  // Use low exposure time to reduce motion blur
//        return setManualExposure(6, 190);  // Use low exposure time to reduce motion blur
////to check
//
//    }


    public boolean april_align(int id) {
        double dis = 0;
        double april_strafe;
//        double s_gain = SPEED_GAIN;


        targetFound = false;
        desiredTag = null;

        if (side_align) {
            id = id + 2;
            april_strafe = april_right_strafe;
        } else april_strafe = april_left_strafe;


        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {

            // Look to see if we have size info on this tag.
            if (detection.metadata != null) {

                //  Check to see if we want to track towards this tag.
                if ((detection.id == id)) {//(DESIRED_TAG_ID < 0) ||
                    // Yes, we want to use this tag.
                    desiredTag = detection;
                    //  if(acal) {ax=desiredTag.ftcPose.x;ay=out_right_dis.getDistance(DistanceUnit.INCH)-10;acal=false;return true;}
                    targetFound = true;
                    break;  // don't look any further.
                }

            }
        }


        if (targetFound) {

            if (dis > 1000) dis = 400;
            rangeError = dis - DESIRED_DISTANCE_T;
            headingError = base_align_angle - imu.getRobotYawPitchRollAngles().getYaw((AngleUnit.DEGREES));
            yawError = desiredTag.ftcPose.bearing + april_strafe;
//                yawError = desiredTag.ftcPose.x - 2.0; //-6.6
            drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);

            strafe = Range.clip(yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
            turn = -Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);


            moveRobot(drive, strafe, turn, -1,true);//outake =-1, intake=1;

        } else {
            driving = true;
            return false;

        }


        return false;
    }

    public void dist_align() {
         gap = front_dist.getDistance(DistanceUnit.MM) - 35;
        power = -0.0008 * gap;
        if (power < -0.4) power = -0.4;
        if (power > -0.02) power = -0.03;
        if (gap >= 9 ) {

            leftFront.setPower(power);
            rightFront.setPower(power);
            leftBack.setPower(power);
            rightBack.setPower(power);

//            motor_vel(leftFront,power,false);
//            motor_vel(rightFront,power,false);
//            motor_vel(leftBack,power,false);
//            motor_vel(rightBack,power,false);

        } else {
            stop_drive();
        }


    }


}


