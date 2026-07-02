package org.firstinspires.ftc.teamcode.autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous
public class MidCycleAutoTetsByTristan extends LinearOpMode {
    static final double FEET_PER_METER = 3.28084;
    public int target = 0;
    double Kp = 0.01;
    boolean isLiftMoving= false;
    int iterations = 0;
    double arm1pos = 0.02;
    double arm2pos = 0.02;
    double clawpos = 0.3;
    double wristpos = 0.91;
    public class Lift {
        DcMotorEx slide1 = hardwareMap.get(DcMotorEx.class, "slide1");
        DcMotorEx slide2 = hardwareMap.get(DcMotorEx.class, "slide2");
        Servo arm1 = hardwareMap.servo.get("arm");
        Servo arm2 = hardwareMap.servo.get("arm2");
        Servo wrist = hardwareMap.servo.get("wrist");
        Servo claw = hardwareMap.servo.get("claw");

        public class MidJunction implements Action{
            public boolean run(@NonNull TelemetryPacket telemetryPacket){
                target = 130;
                arm1pos = 0.96;
                arm2pos = 0.96;
                wristpos = 0.3;
                clawpos = 0.0;
                isLiftMoving = false;
                return false;
            }
        }
        public class DefaultPosition implements Action{
            public boolean run(@NonNull TelemetryPacket telemetryPacket){
                target = 0;
                arm1pos = 0.02;
                arm2pos = 0.02;
                wristpos = 0.91;
                clawpos = 0.0;
                return false;
            }
        }
        public class OpenClaw implements Action{
            public boolean run(@NonNull TelemetryPacket telemetryPacket){
                clawpos = 1;
                return false;
            }
        }
        public class CloseClaw implements Action{
            public boolean run(@NonNull TelemetryPacket telemetryPacket){
                clawpos = 0;
                return false;
            }
        }


        public class Init implements Action{
            public boolean run(@NonNull TelemetryPacket telemetryPacket){
                arm1.setDirection(Servo.Direction.REVERSE);
                arm2.setDirection(Servo.Direction.FORWARD);

                slide1.setDirection(DcMotorSimple.Direction.REVERSE);
                slide2.setDirection(DcMotorSimple.Direction.FORWARD);

                slide1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                slide2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                slide1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                slide2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                arm1.setPosition(0.02);
                arm2.setPosition(0.02);
                wrist.setPosition(0.91);
                claw.setPosition(0.0);

                return false;
            }
        }

        public class GlobalPID implements Action {
            public boolean run(@NonNull TelemetryPacket telemetryPacket){

                int error1 = target - slide1.getCurrentPosition();
                slide1.setPower(-error1 * Kp);
                int error2 = target - slide2.getCurrentPosition();
                slide2.setPower(error2 * Kp);

                arm1.setPosition(arm1pos);
                arm2.setPosition(arm2pos);
                claw.setPosition(clawpos);
                wrist.setPosition(wristpos);

                telemetry.addData("slide1 pos", slide1.getCurrentPosition());
                telemetry.addData("slide2 pos", slide2.getCurrentPosition());
                telemetry.addData("iterations",iterations++);
                telemetry.update();

                return true;
            }
        }

        public class setTarget implements Action{
            int e;
            public setTarget(int a){
                e=a;
            }
            public boolean run(@NonNull TelemetryPacket packet){
                target=e;
                arm1pos=0.02;
                arm2pos=0.02;
                wristpos=0.91;
                return false;
            }
        }


        public Action globalPID() {return new GlobalPID();}
        public Action initialize(){return new Init();}
        public Action midJunction(){return new MidJunction();}
        public Action openClaw(){return new OpenClaw();}

        public Action closeClaw(){return new CloseClaw();}
        public Action defaultPosition(){return new DefaultPosition();}

        public Action setTarget(int z){return new setTarget(z);}
    }

    public void runOpMode() {

        telemetry.setMsTransmissionInterval(50);
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(11, 36, Math.toRadians(0)));

        Action DriveInitialDeposit;
        Action DriveToIntakeFromInitialDeposit;
        Action DriveToIntake;
        Action DriveToDeposit;
        Action ParkZone2;


        //set staring position, unit is inches
        DriveInitialDeposit = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(46,40),Math.toRadians(90))
                .waitSeconds(0.4)
                .build();

        DriveToIntakeFromInitialDeposit = drive.actionBuilder(new Pose2d(46,40,Math.toRadians(90)))
                .strafeTo(new Vector2d(57,36))
                .strafeTo(new Vector2d(56.5,12))
                .waitSeconds(0.2)
                .build();

        DriveToIntake = drive.actionBuilder(new Pose2d(55,39,Math.toRadians(120)))
                .strafeToLinearHeading(new Vector2d(56,12),Math.toRadians(90))
                .waitSeconds(0.2)
                .build();
        Action DriveToIntake2 = drive.actionBuilder(new Pose2d(55,39,Math.toRadians(120)))
                .strafeToLinearHeading(new Vector2d(56,12),Math.toRadians(90))
                .waitSeconds(0.2)
                .build();

        Action DriveToIntake3 = drive.actionBuilder(new Pose2d(55,39,Math.toRadians(120)))
                .strafeToLinearHeading(new Vector2d(56,12),Math.toRadians(90))
                .waitSeconds(0.2)
                .build();
        Action DriveToIntake4 = drive.actionBuilder(new Pose2d(55,39,Math.toRadians(120)))
                .strafeToLinearHeading(new Vector2d(56,12),Math.toRadians(90))
                .waitSeconds(0.2)
                .build();




        DriveToDeposit = drive.actionBuilder(new Pose2d(56,12,Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(55,39),Math.toRadians(125))
                .waitSeconds(0.4)
                .build();

        Action DriveToDeposit2 = drive.actionBuilder(new Pose2d(56,12,Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(55,39),Math.toRadians(125))
                .waitSeconds(0.4)
                .build();

        Action DriveToDeposit3 = drive.actionBuilder(new Pose2d(56,12,Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(55,39),Math.toRadians(125))
                .waitSeconds(0.4)
                .build();
        Action DriveToDeposit4 = drive.actionBuilder(new Pose2d(56,12,Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(55,39),Math.toRadians(125))
                .waitSeconds(0.4)
                .build();
        Action DriveToDeposit5= drive.actionBuilder(new Pose2d(56,12,Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(55,39),Math.toRadians(125))
                .waitSeconds(0.4)
                .build();


        ParkZone2 = drive.actionBuilder(new Pose2d(55,39,Math.toRadians(125)))
                .turn(Math.toRadians(-125))
                .build();


        waitForStart();

        if (isStopRequested()) return;

        Action parkingZone = ParkZone2;
        Lift lift = new Lift();
        Actions.runBlocking(new SequentialAction(
                        lift.initialize(),
                        new ParallelAction(
                                lift.globalPID(),
                                new SequentialAction(

                                        new ParallelAction(
                                                DriveInitialDeposit,
                                                lift.midJunction()
                                        ),

                                        //new SleepAction(0.2),
                                        lift.openClaw(),
                                        new SleepAction(0.2),

                                        new ParallelAction(
                                                lift.setTarget(150),
                                                DriveToIntakeFromInitialDeposit
                                        ),

                                        //new SleepAction(0.2),
                                        lift.closeClaw(),
                                        new SleepAction(0.3),

                                        new ParallelAction(
                                                DriveToDeposit,
                                                new SequentialAction(
                                                        lift.setTarget(300),
                                                        new SleepAction(0.5),
                                                        lift.midJunction()
                                                )

                                        ),

                                        //new SleepAction((0.2)),

                                        lift.openClaw(),
                                        new SleepAction(0.2),

                                        new ParallelAction(
                                                lift.setTarget(120),
                                                DriveToIntake
                                        ),

                                        new SleepAction(0.2),
                                        lift.closeClaw(),
                                        new SleepAction(0.3),

                                        new ParallelAction(
                                                DriveToDeposit2,
                                                new SequentialAction(
                                                        lift.setTarget(300),
                                                        new SleepAction(0.5),
                                                        lift.midJunction()
                                                )

                                        ),

                                        //new SleepAction((0.2)),

                                        lift.openClaw(),
                                        new SleepAction(0.2),

                                        new ParallelAction(
                                                lift.setTarget(85),
                                                DriveToIntake2
                                        ),

                                        new SleepAction(0.2),
                                        lift.closeClaw(),
                                        new SleepAction(0.3),

                                        new ParallelAction(
                                                DriveToDeposit3,
                                                new SequentialAction(
                                                        lift.setTarget(300),
                                                        new SleepAction(0.5),
                                                        lift.midJunction()
                                                )

                                        ),

                                        // new SleepAction((0.2)),

                                        lift.openClaw(),
                                        new SleepAction(0.2),

                                        new ParallelAction(
                                                lift.setTarget(40),
                                                DriveToIntake3
                                        ),

                                        new SleepAction(0.2),
                                        lift.closeClaw(),
                                        new SleepAction(0.3),

                                        new ParallelAction(
                                                DriveToDeposit4,
                                                new SequentialAction(
                                                        lift.setTarget(300),
                                                        new SleepAction(0.5),
                                                        lift.midJunction()
                                                )

                                        ),

                                        //new SleepAction((0.2)),

                                        lift.openClaw(),
                                        new SleepAction(0.2),

                                        new ParallelAction(
                                                lift.setTarget(0),
                                                DriveToIntake4
                                        ),

                                        new SleepAction(0.2),
                                        lift.closeClaw(),
                                        new SleepAction(0.3),

                                        new ParallelAction(
                                                DriveToDeposit5,
                                                new SequentialAction(
                                                        lift.setTarget(300),
                                                        new SleepAction(0.5),
                                                        lift.midJunction()
                                                )

                                        ),

                                        lift.openClaw(),
                                        new SleepAction(0.2),


                                        new ParallelAction(
                                                parkingZone,
                                                lift.defaultPosition()
                                        )



                                )
                        )
                )

        );

        while (opModeIsActive()) {
            sleep(20);
        }
    }
}





