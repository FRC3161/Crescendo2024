package frc.lib.util;

import java.util.HashMap;
import java.util.Map;

/**
 * Class representing a QR decomposition. This class uses the Gram-Schmidt
 * method. For more information on the algorithm
 * used, see:
 * http://www.math.ucla.edu/~yanovsky/Teaching/Math151B/handouts/GramSchmidt.pdf
 * 
 * @author gorosgobe
 */
public class QRDecomposition {

  /** The matrix to decompose */
  private double[][] matrix;
  /** The orthogonal matrix from the decomposition */
  private double[][] Q;
  /** The upper triangular matrix from the decomposition */
  private double[][] R;
  /**
   * A map containing the values used in the Gram-Schmidt algorithm for the
   * orthogonal matrix. Used for memoization
   */
  private Map<Integer, double[][]> EMap;
  /**
   * A map containing the intermediate values used in the Gram-Schmidt algorithm
   * for the orthogonal matrix. Used for memoization
   */
  private Map<Integer, double[][]> UMap;

  /**
   * Performs a QR decomposition on the matrix given
   * 
   * @param matrix the matrix to decompose
   */
  public QRDecomposition(double[][] matrix) {
    // pre: matrix is invertible
    this.matrix = matrix;
    allocateMatrices();
    this.EMap = new HashMap<>();
    this.UMap = new HashMap<>();
    computeDecomposition();
  }

  /**
   * Allocates space for the Q and R matrices from the decomposition
   */
  private void allocateMatrices() {
    this.Q = new double[matrix.length][matrix[0].length];
    this.R = new double[matrix[0].length][matrix[0].length];
  }

  /**
   * Computes the QR decomposition
   */
  private void computeDecomposition() {

    for (int i = 0; i < Q[0].length; i++) {
      putQColumnVectorAtIndex(getEVector(i), i);
    }

    for (int i = 0; i < R.length; i++) {
      for (int j = 0; j < R[0].length; j++) {
        putATimesEValueAtIndices(multiplyDotProductVectors(getEVector(i), getColumnVector(matrix, j)), i, j);
      }
    }

  }

  /**
   * Computes the L2 (Euclidean) norm of a column vector
   * 
   * @param columnVector the column vector to compute the norm of
   * @return the norm of the column vector
   */
  private double computeNorm(double[][] columnVector) {
    double value = 0.0;
    for (int i = 0; i < columnVector.length; i++) {
      value += Math.pow(columnVector[i][0], 2);
    }

    return Math.sqrt(value);
  }

  /**
   * Gets the column vector given by the index supplied from the matrix given
   * 
   * @param matrix the matrix to get the column vector from
   * @param index  the index of the column vector
   * @return the column vector specified by the index in the matrix
   */
  private static double[][] getColumnVector(double[][] matrix, int index) {
    double[][] columnVector = new double[matrix.length][1];

    for (int i = 0; i < matrix.length; i++) {
      columnVector[i][0] = matrix[i][index];
    }

    return columnVector;
  }

  /**
   * Divides every component of a column vector by the divisor given.
   * 
   * @param vector  the column vector to divide each component of
   * @param divisor the divisor to divide each component by
   * @return the column vector with its components divided by the divisor supplied
   */
  private static double[][] divideColumnVector(double[][] vector, double divisor) {

    double[][] result = new double[vector.length][vector[0].length];

    for (int i = 0; i < vector.length; i++) {
      result[i][0] = vector[i][0] / divisor;
    }
    return result;
  }

  /**
   * Computes the dot product of two column vectors
   * 
   * @param vector1 the first column vector
   * @param vector2 the second column vector
   * @return the dot product of both vectors
   */
  private static double multiplyDotProductVectors(double[][] vector1, double[][] vector2) {
    assert vector1.length == vector2.length && vector1[0].length == vector2[0].length : "Dimensions must be equal";
    double result = 0.0;

    for (int i = 0; i < vector1.length; i++) {
      result += vector1[i][0] * vector2[i][0];
    }

    return result;
  }

  /**
   * Multiplies the given column vector by the supplied scalar
   * 
   * @param vector the column vector to multiply
   * @param scalar the scalar to multiply the vector by
   * @return the scaled column vector
   */
  private static double[][] multiplyVectorByScalar(double[][] vector, double scalar) {
    double[][] result = new double[vector.length][vector[0].length];

    for (int i = 0; i < vector.length; i++) {
      result[i][0] = vector[i][0] * scalar;
    }
    return result;
  }

  /**
   * Adds to column vectors.
   * 
   * @param vector1 the first column vector
   * @param vector2 the second column vector
   * @return the column vector resulting from adding both supplied vectors
   */
  private static double[][] addVectors(double[][] vector1, double[][] vector2) {
    assert vector1.length == vector2.length && vector1[0].length == vector2[0].length : "Dimensions must be equal";
    double[][] result = new double[vector1.length][vector1[0].length];

    for (int i = 0; i < vector1.length; i++) {
      result[i][0] = vector1[i][0] + vector2[i][0];
    }

    return result;
  }

  /**
   * Subtracts two column vectors
   * 
   * @param vector1 the first column vector
   * @param vector2 the second column vector
   * @return the column vector resulting from subtracting both supplied vectors
   */
  private static double[][] subtractVectors(double[][] vector1, double[][] vector2) {
    assert vector1.length == vector2.length && vector1[0].length == vector2[0].length : "Dimensions must be equal";
    double[][] result = new double[vector1.length][vector1[0].length];

    for (int i = 0; i < vector1.length; i++) {
      result[i][0] = vector1[i][0] - vector2[i][0];
    }

    return result;
  }

  /**
   * Computes the column vector at the supplied index for the Q matrix, which is
   * also used to compute the entries of the
   * R matrix. This is done by the Gram-Schmidt algorithm.
   * 
   * @param index the index of the column vector of the Q matrix that we want to
   *              obtain
   * @return the column vector of the Q matrix at the supplied index
   */
  private double[][] getEVector(int index) {
    if (EMap.containsKey(index)) {
      return copyOf(EMap.get(index));
    } else {
      double[][] UVector = getIntermediateUVector(index);
      double[][] EVector = divideColumnVector(UVector, computeNorm(UVector));
      EMap.put(index, EVector);
      return EVector;
    }
  }

  /**
   * Computes the intermediate vector used to compute the column vectors in the Q
   * matrix.
   * 
   * @param index the index of the intermediate vector to compute
   * @return the intermediate vector used to compute the column vectors in the Q
   *         matrix with the specified index
   */
  private double[][] getIntermediateUVector(int index) {
    if (index == 0) {
      // return a1
      if (UMap.containsKey(index)) {
        return copyOf(UMap.get(index));
      }

      return getColumnVector(matrix, index);

    } else {

      if (UMap.containsKey(index)) {
        return copyOf(UMap.get(index));
      }

      double[][] uVector = getCompleteUSection(index);
      UMap.put(index, uVector);
      return uVector;
    }

  }

  /**
   * Computes a section of the equation to compute the UVectors used to compute
   * the column vector of the Q matrix.
   * 
   * @param aIndex the index of the column vector from the original matrix to use
   *               in the computation
   * @param eIndex the index of the column vector computed before for the Q matrix
   * @return the section of the equation to compute the UVectors used in the
   *         computation of the column vectors of the
   *         Q matrix, specified by the indices of the vector columns in the
   *         original matrix and in the Q matrix
   */
  private double[][] getUSection(int aIndex, int eIndex) {
    double[][] eVector = getEVector(eIndex);
    return multiplyVectorByScalar(eVector, multiplyDotProductVectors(getColumnVector(matrix, aIndex), eVector));
  }

  /**
   * Uses <em>getUSection</em> to compute the UVector used to compute the column
   * vectors of the Q matrix through the
   * Gram-Schmidt method, associated to the index provided.
   * 
   * @param uIndex the index of the column vector to compute.
   * @return the uVector to compute the column vector of the Q matrix with the
   *         associated index.
   */
  private double[][] getCompleteUSection(int uIndex) {

    double[][] result = getColumnVector(matrix, uIndex);

    for (int i = 0; i < uIndex; i++) {
      result = subtractVectors(result, getUSection(uIndex, i));
    }

    return result;
  }

  /**
   * Copies the vector/matrix supplied
   * 
   * @param vector the column vector to copy (although it can also be a matrix,
   *               for the purposes of this algorithm,
   *               we only use it for column vectors, hence the naming).
   * @return the copy of the vector/matrix supplied.
   */
  private double[][] copyOf(double[][] vector) {
    double[][] result = new double[vector.length][vector[0].length];

    for (int i = 0; i < vector.length; i++) {
      for (int j = 0; j < vector[0].length; j++) {
        result[i][j] = vector[i][j];
      }
    }

    return result;
  }

  /**
   * Puts the given column vector at the column specified by the index of the Q
   * matrix.
   * 
   * @param vector the column vector to put in the matrix Q.
   * @param index  the index to put the column vector at.
   */
  private void putQColumnVectorAtIndex(double[][] vector, int index) {

    for (int i = 0; i < Q.length; i++) {
      Q[i][index] = vector[i][0];
    }

  }

  /**
   * Static method that solves by back substitution the equation Rb = QTy, where b
   * is the unknown matrix, R is the upper
   * triangular matrix from QR decomposition and QT is the transpose of the
   * orthogonal matrix Q from QR decomposition.
   * 
   * @param r   the upper triangular matrix multiplying the unknowns
   * @param qty the matrix obtained by multiplying the transpose of Q and the
   *            response matrix
   * @return the coefficient matrix
   */
  public static double[][] solveByBackSubstitution(double[][] r, double[][] qty) {
    double[][] result = new double[r[0].length][1];

    for (int i = r[0].length - 1; i >= 0; i--) {
      result[i][0] = qty[i][0];
      for (int j = i + 1; j < r[0].length; j++) {
        result[i][0] = result[i][0] - (r[i][j] * result[j][0]);
      }
      result[i][0] = result[i][0] / r[i][i];
    }

    return result;
  }

  /**
   * Put the value given by the dot product of the column of the original matrix
   * and the column of the Q matrix
   * (eVector) at the specified indices
   * 
   * @param value the value given by the dot product of the column of the original
   *              matrix and the column of the Q matrix
   * @param i     the row index
   * @param j     the column index
   */
  private void putATimesEValueAtIndices(double value, int i, int j) {
    R[i][j] = value;
  }

  /**
   * Gets the original matrix.
   * 
   * @return the original matrix supplied.
   */
  public double[][] getMatrix() {
    return matrix;
  }

  /**
   * Gets the Q orthogonal matrix from the QR decomposition.
   * 
   * @return the Q orthogonal matrix.
   */
  public double[][] getQ() {
    return Q;
  }

  /**
   * Gets the upper triangular matrix R from the QR decomposition.
   * 
   * @return the R upper triangular matrix.
   */
  public double[][] getR() {
    return R;
  }
}
