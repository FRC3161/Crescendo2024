package frc.lib.util;

/**
 * Class representing utils for Matrix operations
 * 
 * @author gorosgobe
 */
public class MatrixUtils {

  /** EPSILON constant used for double comparison and equality */
  public static final double EPSILON = StatisticUtils.EPSILON;

  /**
   * Multiplies two matrices.
   * 
   * @param matrix1 first matrix
   * @param matrix2 second matrix
   * @return the resulting matrix from multiplying the first matrix with the
   *         second matrix.
   */
  public static double[][] multiply(double[][] matrix1, double[][] matrix2) {
    assert matrix1[0].length == matrix2.length : "Dimensions must match";

    double[][] result = new double[matrix1.length][matrix2[0].length];

    for (int i = 0; i < matrix1.length; i++) {
      for (int j = 0; j < matrix2[0].length; j++) {
        for (int k = 0; k < matrix1[0].length; k++) {
          result[i][j] = result[i][j] + matrix1[i][k] * matrix2[k][j];
        }
      }
    }

    return result;
  }

  /**
   * Transposes the given matrix.
   * 
   * @param matrix the matrix to transpose
   * @return the transposed matrix
   */
  public static double[][] transpose(double[][] matrix) {
    double[][] transposed = new double[matrix[0].length][matrix.length];

    for (int i = 0; i < matrix.length; i++) {
      for (int j = 0; j < matrix[0].length; j++) {
        transposed[j][i] = matrix[i][j];
      }
    }

    return transposed;
  }

  /**
   * Inverts the given matrix using the QR decomposition based method. A = QR
   * therefore A-1 = (QR)-1, so A-1 = R-1Q-1
   * which, given that Q is orthogonal (QT x Q = I, QT = Q-1), gives <em>A-1 =
   * R-1QT</em>. A precondition is that the matrix
   * is invertible.
   * 
   * @param matrix the matrix to invert
   * @return the inverted matrix
   */
  public static double[][] inverse(double[][] matrix) {
    // QRDecomposition based method
    assert matrix.length == matrix[0].length : "Matrix has to have nxn entries";

    if (matrix.length == 1) {
      double[][] result = new double[1][1];
      result[0][0] = 1 / matrix[0][0];
      return result;
    }

    // all inverses dealt with are 2x2 or bigger
    QRDecomposition decomposition = new QRDecomposition(matrix);

    double[][] R = decomposition.getR();

    // corrects numbers due to numerical stability, setting the lower part to zero
    for (int i = 1; i < R.length; i++) {
      for (int j = 0; j < i; j++) {
        R[i][j] = 0;
      }
    }

    double[][] QT = MatrixUtils.transpose(decomposition.getQ());
    double[][] RInverse = inverseForUpperTriangular(R);

    return MatrixUtils.multiply(RInverse, QT);
  }

  /**
   * Helper used by the <em>inverse</em> method. Inverts the upper triangular
   * matrix R from the QR decomposition.
   * 
   * @param matrix the upper triangular matrix to invert
   * @return the inverted matrix
   */
  private static double[][] inverseForUpperTriangular(double[][] matrix) {
    assert matrix.length == matrix[0].length : "Matrix has to have nxn entries";

    double[][] workingMatrix = new double[matrix.length][2 * matrix[0].length];

    // initialises identity matrix in right part of working matrix
    for (int i = 0; i < workingMatrix.length; i++) {
      workingMatrix[i][i + workingMatrix[0].length / 2] = 1;
    }

    // initialises left part of working matrix to be the given matrix
    for (int i = 0; i < matrix.length; i++) {
      System.arraycopy(matrix[i], 0, workingMatrix[i], 0, matrix[0].length);
    }

    // creates ones in the diagonal
    for (int i = 0; i < matrix.length; i++) {
      workingMatrix = multiplyRow(workingMatrix, i, 1 / workingMatrix[i][i]);
    }

    // corrects numbers due to numerical stability, setting the lower part to zero
    for (int i = 1; i < workingMatrix.length; i++) {
      for (int j = 0; j < i; j++) {
        workingMatrix[i][j] = 0;
      }
    }

    // puts the left part of working matrix in RREF (reduced row echelon form)
    // starts by ignoring the first column as that one is already in RREF (matrix
    // inputted was at least 2x2)
    for (int i = 0; i < matrix.length; i++) {
      for (int j = i + 1; j < matrix.length; j++) {
        workingMatrix = subtractRows(workingMatrix, i, j, workingMatrix[i][j]);
      }
    }

    // selects right part of working matrix
    double[][] result = new double[matrix.length][matrix[0].length];

    for (int i = 0; i < matrix.length; i++) {
      for (int j = workingMatrix[0].length / 2; j < workingMatrix[0].length; j++) {
        result[i][j - workingMatrix[0].length / 2] = workingMatrix[i][j];
      }
    }

    return result;
  }

  /**
   * Naive implementation of the inverse of a matrix using the general method
   * based on Gaussian Elimination. A precondition
   * is that the matrix given is invertible.
   * 
   * @param matrix the matrix to invert
   * @return the inverted matrix
   */
  public static double[][] naiveInverse(double[][] matrix) {
    assert matrix.length == matrix[0].length : "Matrix has to have nxn entries";

    if (matrix.length == 1) {
      double[][] result = new double[1][1];
      result[0][0] = 1 / matrix[0][0];
      return result;
    }

    // all inverses dealt with are 2x2 or bigger
    // naive and not most efficient method for computing an inverse of a matrix
    double[][] workingMatrix = new double[matrix.length][2 * matrix[0].length];

    // initialises identity matrix in right part of working matrix
    for (int i = 0; i < workingMatrix.length; i++) {
      workingMatrix[i][i + workingMatrix[0].length / 2] = 1;
    }

    // initialises left part of working matrix to be the given matrix
    for (int i = 0; i < matrix.length; i++) {
      System.arraycopy(matrix[i], 0, workingMatrix[i], 0, matrix[0].length);
    }

    // converts matrix in upper triangular with 1s in diagonal
    for (int i = 0; i < matrix.length; i++) {
      for (int j = i + 1; j < matrix.length; j++) {
        workingMatrix = subtractRows(workingMatrix, j, i, workingMatrix[j][i] / workingMatrix[i][i]);
      }
    }

    for (int i = 0; i < matrix.length; i++) {
      workingMatrix = multiplyRow(workingMatrix, i, 1 / workingMatrix[i][i]);
    }

    // puts the left part of working matrix in RREF (reduced row echelon form)
    // starts by ignoring the first column as that one is already in RREF (matrix
    // inputted was at least 2x2)
    for (int i = 0; i < matrix.length; i++) {
      for (int j = i + 1; j < matrix.length; j++) {
        workingMatrix = subtractRows(workingMatrix, i, j, workingMatrix[i][j]);
      }
    }

    // selects right part of working matrix
    double[][] result = new double[matrix.length][matrix[0].length];

    for (int i = 0; i < matrix.length; i++) {
      for (int j = workingMatrix[0].length / 2; j < workingMatrix[0].length; j++) {
        result[i][j - workingMatrix[0].length / 2] = workingMatrix[i][j];
      }
    }

    return result;
  }

  /**
   * Helper method for the invert functions that subtracts a row multiplied by a
   * factor from another row of the supplied
   * matrix
   * 
   * @param matrix    the matrix to get the rows from
   * @param firstRow  the index of the first row
   * @param secondRow the index of the second row
   * @param factor    the factor to multiply the second row to
   * @return the matrix with the row subtraction operation completed
   */
  public static double[][] subtractRows(double[][] matrix, int firstRow, int secondRow, double factor) {
    assert !(firstRow == secondRow) : "both rows must not be the same";
    double[][] result = new double[matrix.length][matrix[0].length];

    // initialises result matrix except target row to modify from given matrix
    for (int i = 0; i < matrix.length; i++) {

      if (i == firstRow) {
        continue;
      }

      System.arraycopy(matrix[i], 0, result[i], 0, matrix[0].length);
    }

    for (int j = 0; j < matrix[0].length; j++) {
      result[firstRow][j] = matrix[firstRow][j] - matrix[secondRow][j] * factor;
    }

    return result;
  }

  /**
   * Multiplies a row of the supplied matrix by a factor.
   * 
   * @param matrix the matrix to multiply a row of
   * @param row    the index of the row to multiply
   * @param factor the factor to multiply the row by
   * @return the matrix with the row multiplied by a factor.
   */
  public static double[][] multiplyRow(double[][] matrix, int row, double factor) {
    double[][] result = new double[matrix.length][matrix[0].length];

    for (int i = 0; i < matrix.length; i++) {
      if (i == row) {
        continue;
      }

      System.arraycopy(matrix[i], 0, result[i], 0, matrix[0].length);
    }

    for (int j = 0; j < matrix[0].length; j++) {
      result[row][j] = matrix[row][j] * factor;
    }

    return result;

  }

  /**
   * Method that compares two matrices to check equality.
   * 
   * @param matrix1 the first matrix
   * @param matrix2 the second matrix
   * @param epsilon the epsilon to use for comparison
   * @return are both matrices equal within the epsilon given?
   */
  public static boolean areMatricesApproximatelyEqual(double[][] matrix1, double[][] matrix2, double epsilon) {
    assert matrix1.length == matrix2.length && matrix1[0].length == matrix2[0].length : "Dimensions must be equal";

    for (int i = 0; i < matrix1.length; i++) {
      for (int j = 0; j < matrix1[0].length; j++) {
        if (!StatisticUtils.isApproxEqual(matrix1[i][j], matrix2[i][j], epsilon)) {
          // not equal therefore return false
          return false;
        }
      }
    }
    return true;
  }

  /**
   * Method that compares two matrices using the default StatisticUtils.EPSILON as
   * epsilon
   * 
   * @param matrix1 the first matrix
   * @param matrix2 the second matrix
   * @return are both matrices equal within StatisticUtils.EPSILON?
   */
  public static boolean areMatricesApproximatelyEqual(double[][] matrix1, double[][] matrix2) {
    return areMatricesApproximatelyEqual(matrix1, matrix2, EPSILON);
  }

  /**
   * Prints the given matrix. Used for debugging
   * 
   * @param matrix the matrix to print
   */
  public static void printMatrix(double[][] matrix) {
    for (int i = 0; i < matrix.length; i++) {
      for (int j = 0; j < matrix[0].length; j++) {
        System.out.print(matrix[i][j] + " ");
      }
      System.out.println();
    }
  }

}
