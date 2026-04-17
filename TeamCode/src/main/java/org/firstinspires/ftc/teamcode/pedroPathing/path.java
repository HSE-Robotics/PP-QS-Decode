package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 *
 * @author Gerry DLIII - 18908 Mighty Hawks
 * @version 1.0, 02/11/2024
 */
    @Autonomous(name = "Reggie_Auto_RED", group = "LM1 Reggie")
    public class path extends OpMode {
        //private static final Logger log = LoggerFactory.getLogger(AyCrush2P_PP.class);
        //Pedro Pathing Variables
        private Follower follower;
        private Reggie Miller;
        public static Pose startingPose;
        public int motif;
        private Timer pathTimer, actionTimer, opmodeTimer;
        private int pathState;
        private Path Start;
        private PathChain score, toSpike2, getSpike2, openGate, gate2Score, score2Gate, gate2ST, sT2Score;
        private final Pose startPose = new Pose(33.4, 134.6, Math.toRadians(90));
        private final Pose scorePose = new Pose(44.7, 102.8, Math.toRadians(180));
        private final Pose aimSpike2 = new Pose(42, 75, Math.toRadians(180));
        private final Pose pickupSpike2 = new Pose(20, 75, Math.toRadians(180));
        private final Pose gate = new Pose(17.2, 81.1, Math.toRadians(180));
        private final Pose gateControl = new Pose(44.7, 82.3, Math.toRadians(180));
        private final Pose scoreGateControl = new Pose(61.1, 78.6, Math.toRadians(180));
        private final Pose secretTunnel = new Pose(12.9, 60.8, Math.toRadians(130));
        private final Pose secretTunnelControl = new Pose(27.3, 65.4, Math.toRadians(130));






        /**
         * This initializes the drive motors as well as the Follower and motion Vectors.
         */
        //@Override


        @Override
        public void init() {
            //Pedro Pathing
//        Constants.setConstants(FConstants.class, LConstants.class);
            follower = Constants.createFollower(hardwareMap);
            follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
            follower.update();
            Miller = new Reggie();
            Miller.init(hardwareMap);

            pathTimer = new Timer();
            opmodeTimer = new Timer();
            opmodeTimer.resetTimer();
            follower = Constants.createFollower(hardwareMap);
            buildPaths();
            follower.setStartingPose(startPose);
            //motif = order(Miller.huskyLens);

            // playTime.reset();
        }

        public void buildPaths() {
            /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
            score = follower.pathBuilder()
                    .addPath(new BezierLine(startPose, scorePose))
                    .setConstantHeadingInterpolation(scorePose.getHeading())
                    .build();
            /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
            toSpike2 = follower.pathBuilder()
                    .addPath(new BezierLine(scorePose, aimSpike2))
                    .setConstantHeadingInterpolation(aimSpike2.getHeading())
                    .build();
            getSpike2 = follower.pathBuilder()
                    .addPath(new BezierLine(aimSpike2,pickupSpike2))
                    .setConstantHeadingInterpolation(pickupSpike2.getHeading())
                    .build();
            openGate = follower.pathBuilder()
                    .addPath(new BezierCurve(pickupSpike2, gateControl, gate))
                    .setConstantHeadingInterpolation(gate.getHeading())
                    .build();
            gate2Score = follower.pathBuilder()
                    .addPath(new BezierCurve(gate,scoreGateControl,scorePose))
                    .setConstantHeadingInterpolation(scorePose.getHeading())
                    .build();
            score2Gate = follower.pathBuilder()
                    .addPath(new BezierCurve(scorePose,scoreGateControl,gate))
                    .setConstantHeadingInterpolation(gate.getHeading())
                    .build();
            gate2ST = follower.pathBuilder()
                    .addPath(new BezierCurve(gate,secretTunnelControl,secretTunnel))
                    .setConstantHeadingInterpolation(secretTunnel.getHeading())
                    .build();
            sT2Score = follower.pathBuilder()
                    .addPath(new BezierCurve(secretTunnel, gateControl, scorePose))
                    .setConstantHeadingInterpolation(scorePose.getHeading())
                    .build();
            /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        }

        public void autonomousPathUpdate() {
            switch (pathState) {
                case 0:
                    follower.setMaxPower(.9);
                    follower.followPath(score,true);
                    setPathState(1);

                    //Miller.shootClose();
                    if(pathTimer.getElapsedTimeSeconds() >= 6){

                        break;
                    }
                    break;
                case 11:
                    if(pathTimer.getElapsedTimeSeconds() >= 2){

                        setPathState(1);
                    }
                    break;
                case 1:
                    follower.setMaxPower(.8);

                    /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                    if(!follower.isBusy()) {
                        /* Score Preload */
                        /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                        follower.followPath(toSpike2,true);
//                    Miller.setShooterPower(100);
                        setPathState(2);
                    }
                    break;
                case 2:
                    if(!follower.isBusy()) {
                        follower.followPath(getSpike2);
                        setPathState(3);
                    }
                    break;
                case 3:
                    if (!follower.isBusy()){
                        follower.followPath(openGate);
                        setPathState(4);
                    }
                    break;
                case 4:
                    if(!follower.isBusy()){
                        follower.followPath(gate2Score);
                        setPathState(5);
                    }
                    break;
                case 5:
                    if (!follower.isBusy()){
                        follower.followPath(score2Gate);
                        setPathState(6);
                    }
                    break;
                case 6:
                    if (!follower.isBusy()){
                        follower.followPath(gate2ST);
                        setPathState(7);
                    }
                    break;
                case 7:
                    if (!follower.isBusy()){
                        follower.followPath(sT2Score);
                        setPathState(8);
                    }
                    break;
                case 8:
                    if(!follower.isBusy()){
                        requestOpModeStop();
                    }
            }
        }
        /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
        public void setPathState(int pState) {
            pathState = pState;
            pathTimer.resetTimer();
        }
        /**
         * This method is called once at the start of the OpMode.
         **/
        @Override
        public void start() {



        }

        /**
         * This runs the OpMode. This is only drive control with Pedro Pathing live centripetal force
         * correction.
         */
        @Override
        public void loop() {
            /**Pedro Pathing Driving
             *
             */

            follower.update();
            autonomousPathUpdate();
            // Feedback to Driver Hub for debugging
            telemetry.addData("motif", motif);
            telemetry.addData("path state", pathState);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.update();

        }
    }

