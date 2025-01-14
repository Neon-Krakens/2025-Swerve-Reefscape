package frc.robot.math;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class CoordinateSystems {

    public static final int[][] LeftSide_FieldToRob = { { 0, -1 }, { 1, 0 } };
    public static final int[][] LeftSide_RobToField = { { 0, 1 }, { -1, 0 } };
    public static final int[][] RightSide_FieldToRob = { { -1, 0 }, { -1, 0 } };
    public static final int[][] RightSide_RobToField = { { -1, 0 }, { -1, 0 } };

    public static Pose2d LeftSide_RobToField(Pose2d currentpose) {

        double[] randomshit = { currentpose.getX(), currentpose.getY() };

        double[] myDubski = multiply(LeftSide_RobToField, randomshit);

        Translation2d trans = new Translation2d(myDubski[0], myDubski[1]);

        return new Pose2d(trans, currentpose.getRotation());

    }

    public static Pose2d LeftSide_FieldToRob(Pose2d currentpose) {

        double[] randomshit = { currentpose.getX(), currentpose.getY() };

        double[] myDubski = multiply(LeftSide_FieldToRob, randomshit);

        Translation2d trans = new Translation2d(myDubski[0], myDubski[1]);

        return new Pose2d(trans, currentpose.getRotation());

    }

    public static Pose2d RightSide_RobToField(Pose2d currentpose) {

        double[] randomshit = { currentpose.getX(), currentpose.getY() };

        double[] myDubski = multiply(RightSide_RobToField, randomshit);

        Translation2d trans = new Translation2d(myDubski[0], myDubski[1]);

        return new Pose2d(trans, currentpose.getRotation());

    }

    public static Pose2d RightSide_FieldToRob(Pose2d currentpose) {

        double[] randomshit = { currentpose.getX(), currentpose.getY() };

        double[] myDubski = multiply(RightSide_FieldToRob, randomshit);

        Translation2d trans = new Translation2d(myDubski[0], myDubski[1]);

        return new Pose2d(trans, currentpose.getRotation());

    }

    public static Pose2d FieldMiddle_FieldBottomLeft(Pose2d middlePose) {

        double x = middlePose.getX() + 8.27;
        double y = middlePose.getY() + 4.105;

        return new Pose2d(x, y, middlePose.getRotation());
    }

    public static double[] multiply(int[][] matrix, double[] vector) {
        int rows = matrix.length;
        int cols = matrix[0].length;

        double[] result = new double[rows];

        for (int i = 0; i < rows; i++) {
            double sum = 0;
            for (int j = 0; j < cols; j++) {
                sum += matrix[i][j] * vector[j];
            }
            result[i] = sum;
        }

        return result;
    }

}
