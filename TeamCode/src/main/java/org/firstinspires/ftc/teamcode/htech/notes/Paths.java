package org.firstinspires.ftc.teamcode.htech.notes;

import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;




class Paths {

    public Paths() {
        //start pose 135, 83
        PathBuilder specimen_auto = new PathBuilder();//
        specimen_auto
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(135.000, 83.000, Point.CARTESIAN),
                                new Point(108.500, 83.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 2
                        new BezierCurve(
                                new Point(108.500, 83.000, Point.CARTESIAN),
                                new Point(160.000, 91.334, Point.CARTESIAN),
                                new Point(1.898, 126.208, Point.CARTESIAN),
                                new Point(134.511, 118.616, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setReversed(true)
                .addPath(
                        // Line 3
                        new BezierCurve(
                                new Point(134.511, 118.616, Point.CARTESIAN),
                                new Point(21.351, 123.124, Point.CARTESIAN),
                                new Point(134.036, 129.292, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 4
                        new BezierCurve(
                                new Point(134.036, 129.292, Point.CARTESIAN),
                                new Point(15.000, 140.000, Point.CARTESIAN),
                                new Point(135.000, 135.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 5
                        new BezierCurve(
                                new Point(135.000, 135.000, Point.CARTESIAN),
                                new Point(110.000, 132.000, Point.CARTESIAN),
                                new Point(143.000, 75.000, Point.CARTESIAN),
                                new Point(108.500, 79.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setReversed(true)
                .addPath(
                        // Line 6
                        new BezierCurve(
                                new Point(108.500, 79.000, Point.CARTESIAN),
                                new Point(143.000, 75.000, Point.CARTESIAN),
                                new Point(110.000, 132.000, Point.CARTESIAN),
                                new Point(135.000, 135.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0));


        //start pose 135, 60
        PathBuilder basket_specimen = new PathBuilder();//
        basket_specimen
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(135.000, 60.000, Point.CARTESIAN),
                                new Point(108.500, 60.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 2
                        new BezierCurve(
                                new Point(108.500, 60.000, Point.CARTESIAN),
                                new Point(128.000, 50.000, Point.CARTESIAN),
                                new Point(105.000, 23.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(105.000, 23.000, Point.CARTESIAN),
                                new Point(121.000, 21.500, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(120))
                .addPath(
                        // Line 4
                        new BezierCurve(
                                new Point(121.000, 21.500, Point.CARTESIAN),
                                new Point(107.466, 131.901, Point.CARTESIAN),
                                new Point(130.000, 125.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(0))
                .addPath(
                        // Line 5
                        new BezierCurve(
                                new Point(130.000, 125.000, Point.CARTESIAN),
                                new Point(105.094, 125.000, Point.CARTESIAN),
                                new Point(140.442, 73.000, Point.CARTESIAN),
                                new Point(108.500, 73.000, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed(true);
    }

}

