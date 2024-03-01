package frc.lib.util;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.*;
import java.util.stream.IntStream;

/**
 * Class representing a polynomial regression on the supplied training data.
 * Allows efficient multithreaded computation
 * of optimal polynomial degree for trained data and supplied test data
 * 
 * @author gorosgobe
 */
public class PolynomialRegression {

  /** List of points representing the training data */
  private final List<Point> points;
  /** Desired polynomial degree for the regression */
  private final int polynomialDegree;
  /** Matrix representing the computed coefficients */
  private double[][] coefficients;

  /*
   * Parameter column vector beta = (XTX)-1XTy where X is the design matrix.
   * Polynomial degree is number of columns of matrix
   */

  /**
   * Constructor of a polynomial regression.
   * 
   * @param points           the training data
   * @param polynomialDegree the desired degree of the polynomial regression
   */
  public PolynomialRegression(List<Point> points, int polynomialDegree) {
    this.points = points;
    if (polynomialDegree < 0) {
      throw new IllegalArgumentException("Polynomial degree should be >= 0");
    }
    this.polynomialDegree = polynomialDegree;
    computeCoefficients();
  }

  /**
   * Constructor of a polynomial regression.
   * 
   * @param points           the training data
   * @param polynomialDegree the desired degree of the polynomial regression
   * @param computeCoeff     should coefficients be computed
   */
  private PolynomialRegression(List<Point> points, int polynomialDegree, boolean computeCoeff) {
    this.points = points;
    if (polynomialDegree < 0) {
      throw new IllegalArgumentException("Polynomial degree should be >= 0");
    }

    this.polynomialDegree = polynomialDegree;
    if (computeCoeff) {
      computeCoefficients();
    }
  }

  /**
   * Gets the training data
   * 
   * @return a list containing the training data as Points
   */
  public List<Point> getPoints() {
    return points;
  }

  /**
   * Gets the degree of the polynomial regression.
   * 
   * @return the degree of the polynomial regression.
   */
  public int getPolynomialDegree() {
    return polynomialDegree;
  }

  /**
   * Gets the coefficients of the polynomial regression. Will return null if the
   * coefficients havent been computed yet.
   * 
   * @return the coefficient matrix.
   */
  public double[][] getCoefficients() {
    return coefficients;
  }

  /**
   * Generates the design matrix with the training data.
   * 
   * @return the design matrix of the training data supplied in the constructor.
   */
  // Matrix with the x of the points
  private double[][] generateDesignMatrix() {

    // width is polynomial grade + 1 as column 0 is a column of 1s
    double[][] designMatrix = new double[points.size()][polynomialDegree + 1];

    // sets first column to be 1
    for (int i = 0; i < designMatrix.length; i++) {
      designMatrix[i][0] = 1;
    }

    // fills up the matrix with the points
    for (int i = 0; i < designMatrix.length; i++) {
      for (int j = 1; j < designMatrix[0].length; j++) {
        designMatrix[i][j] = Math.pow(points.get(i).getX(), j);
      }
    }

    return designMatrix;
  }

  /**
   * Generates the response matrix with the training data.
   * 
   * @return the response matrix with the training data.
   */
  // Matrix with the y of the points
  private double[][] generateResponseMatrix() {
    double[][] responseMatrix = new double[points.size()][1];

    for (int i = 0; i < responseMatrix.length; i++) {
      responseMatrix[i][0] = points.get(i).getY();
    }

    return responseMatrix;
  }

  /**
   * Computes the coefficients of the polynomial regression and stores them in the
   * <em>coefficients</em> field.
   */
  private void computeCoefficients() {
    double[][] coefficients;
    double[][] designMatrix = generateDesignMatrix();
    double[][] responseMatrix = generateResponseMatrix();

    // coefficient matrix is given by the equation described above
    // NAIVE AND INEFFICIENT WAY:
    // coefficients = MatrixUtils.multiply(MatrixUtils.multiply(MatrixUtils.inverse(
    // MatrixUtils.multiply(designMatrixTranspose, designMatrix)),
    // designMatrixTranspose), responseMatrix);

    // QR DECOMPOSITION AND BACK SUBSTITUTION
    QRDecomposition decomp = new QRDecomposition(designMatrix);
    coefficients = QRDecomposition.solveByBackSubstitution(decomp.getR(),
        MatrixUtils.multiply(MatrixUtils.transpose(decomp.getQ()), responseMatrix));

    this.coefficients = coefficients;
  }

  /**
   * Gets a prediction for the given value when the regression is trained
   * 
   * @param value the value (x) we want to get a prediction for
   * @return the predicted value for the supplied argument according to the
   *         trained model
   */
  public double getPrediction(double value) {

    if (coefficients == null) {
      computeCoefficients();
    }

    double result = 0.0;

    for (double i = 0.0; i < coefficients.length; i++) {
      result += coefficients[(int) i][0] * Math.pow(value, i);
    }

    return result;
  }

  /**
   * Multithreaded implementation of a method based on Root Mean Square Error
   * (RMSE) comparison to obtain the optimal polynomial
   * degree that minimises the RMSE error, and therefore improves the accuracy of
   * the trained data, given test data.
   * 
   * @param testData       the data we want to optimise the regression for
   * @param terminalOutput do you want the thread ids, errors, polynomial degrees
   *                       and elapsed times to be outputted to
   *                       the standard output?
   * @return the integer representing the optimal degree for the polynomial
   *         regression for the trained data
   * @throws InterruptedException
   */
  public static int getOptimalPolynomialDegreeWithTestData(List<Point> trainingData, List<Point> testData,
      boolean terminalOutput)
      throws InterruptedException {
    if (trainingData.isEmpty()) {
      throw new IllegalArgumentException("Empty training data");
    }

    if (testData.isEmpty()) {
      throw new IllegalArgumentException("Empty test data");
    }

    long startTime = System.nanoTime();

    // gets max number of threads
    int threadNum = Runtime.getRuntime().availableProcessors() - 1;
    // creates an array of threads
    final Thread[] threads = new Thread[threadNum];
    // creates an array of polynomial degrees
    final int[] threadPolyDegrees = new int[threadNum];
    // creates an array of root mean squared errors
    final double[] threadRMSE = new double[threadNum];
    final int[] sequence = new int[testData.size() - 1];

    for (int i = 0; i < testData.size() - 1; i++) {
      sequence[i] = i;
    }

    // creates an even distribution of polynomial degrees so each thread does
    // approximately the same amount of work
    distribute(sequence, threadNum);

    // for every available thread with index: threadIndex
    IntStream.range(0, threadNum).forEach(threadIndex -> {
      // create a new thread
      Thread t = new Thread(() -> {
        // with a mapping from RSMErrors to polynomial degrees
        Map<Double, Integer> errorsToDegree = new LinkedHashMap<>();

        for (int i = (sequence.length * threadIndex) / threadNum; i < (sequence.length * (threadIndex + 1))
            / threadNum; i++) {

          PolynomialRegression regression = new PolynomialRegression(trainingData, sequence[i], true);
          double error = regression.getTestDataRootMeanSquareError(testData);
          if (terminalOutput) {
            System.out.println("Thread: " + threadIndex + ", Degree: " + sequence[i] + ", Error: " + error);
          }
          errorsToDegree.put(error, sequence[i]);

        }

        // for the degree section corresponding to the thread index, compute the minimum
        // error and get
        // its polynomial degree
        threadRMSE[threadIndex] = Collections.min(errorsToDegree.keySet());
        threadPolyDegrees[threadIndex] = errorsToDegree.get(threadRMSE[threadIndex]);

      });

      threads[threadIndex] = t;

    });

    // start threads
    for (Thread t : threads) {
      t.start();
    }
    // wait until the last thread to complete its process
    for (Thread t : threads) {
      t.join();
    }

    int minimalDegreeIndex = getIndexOfMinDouble(threadRMSE);

    long endTime = System.nanoTime();
    if (terminalOutput) {
      System.out.println("Time required: " + ((endTime - startTime) / 1000000000.0) + "s");

    }
    return threadPolyDegrees[minimalDegreeIndex];
  }

  /**
   * Multithreaded implementation of a method based on Root Mean Square Error
   * (RMSE) comparison to obtain the optimal polynomial
   * degree that minimises the RMSE error, and therefore improves the accuracy of
   * the trained data, given test data.
   * By default, it prints to terminal the errors, polynomial degrees, the thread
   * ids and the elapsed time.
   * 
   * @param testData the data we want to optimise the regression for
   * @return the integer representing the optimal degree for the polynomial
   *         regression for the trained data
   * @throws InterruptedException
   */
  public static int getOptimalPolynomialDegreeWithTestData(List<Point> trainingData, List<Point> testData)
      throws InterruptedException {
    return getOptimalPolynomialDegreeWithTestData(trainingData, testData, true);
  }

  /**
   * Returns the optimal polynomial regression (with minimised RMSE) for the
   * supplied training data and test data. Also
   * prints the errors, poly-degrees and time elapsed in the computation. This
   * also includes the thread ids, as this is
   * a multithreaded implementation. Bear in mind, however, that this method is
   * computationally intensive should a lot
   * of testing points be supplied.
   * 
   * @param trainingData   the training data for the regression
   * @param testData       the test data to minimise the RMSE error of
   * @param terminalOutput do you want the thread ids, errors, polynomial degrees
   *                       and elapsed times to be outputted to
   *                       the standard output?
   * @return the optimal Polynomial Regression for the supplied training data and
   *         test data
   * @throws InterruptedException
   */
  public static PolynomialRegression getOptimalPolynomialRegression(List<Point> trainingData, List<Point> testData,
      boolean terminalOutput) throws InterruptedException {
    int optimalDegree = getOptimalPolynomialDegreeWithTestData(trainingData, testData, terminalOutput);
    return new PolynomialRegression(trainingData, optimalDegree);
  }

  /**
   * Splits an array into n smaller arrays containing the supplied size. Used in
   * the distribute function that
   * gives each thread a number of tasks with a similar combined difficulty, so
   * each thread works approximately
   * the same as the other ones, and therefore minimising the Thread.join() time.
   * 
   * @param items           the integer array with the items to split
   * @param maxSubArraySize the max size of the subarrays
   * @return a list with the arrays containing the maxSubArraySize. The last array
   *         will contain maxSubArraySize items
   *         or the remaining ones if the <em>items</em> array size was not
   *         multiple of maxSubArraySize.
   */
  private static List<int[]> splitArray(int[] items, int maxSubArraySize) {
    List<int[]> result = new ArrayList<>();

    if (items == null || items.length == 0) {
      return result;
    }

    int from = 0;
    int to = 0;
    int slicedItems = 0;
    while (slicedItems < items.length) {
      to = from + Math.min(maxSubArraySize, items.length - to);
      int[] slice = Arrays.copyOfRange(items, from, to);
      result.add(slice);
      slicedItems += slice.length;
      from = to;
    }

    return result;
  }

  /**
   * Distributes the tasks, ordered by ascending difficulty in the supplied int
   * array, by rearranging them so the difficulty
   * of each thread's work is similar.
   * 
   * @param array      the array containing the task ids by order of difficulty.
   * @param numThreads the number of threads we will be using
   */
  private static void distribute(int[] array, int numThreads) {

    List<int[]> list = splitArray(array, array.length / numThreads);
    int count = 0;

    for (int i = 0; i < array.length / numThreads; i++) {
      for (int j = 0; j < numThreads; j++) {
        array[count] = list.get(j)[count / numThreads];
        count++;
      }
    }

  }

  /**
   * Gets the index of the minimum double in the supplied array.
   * 
   * @param doubles an array of doubles
   * @return the index of the minimum double in the array
   */
  private static int getIndexOfMinDouble(double[] doubles) {

    double current = Double.MAX_VALUE;
    int currentIndex = 0;

    for (int i = 0; i < doubles.length; i++) {
      if (doubles[i] < current) {
        current = doubles[i];
        currentIndex = i;
      }
    }

    return currentIndex;

  }

  /**
   * Gets the RSME (Root Mean Square Error) of the training data.
   * 
   * @return the RSME of the training data.
   */
  public double getTrainingDataRootMeanSquareError() {
    double meanSquareError = points.stream()
        .map(i -> Math.pow(getPrediction(i.getX()) - i.getY(), 2.0))
        .reduce(0.0, (a, b) -> a + b) / points.size();

    return Math.sqrt(meanSquareError);
  }

  /**
   * Gets the RSME (Root Mean Square Error) of the test data. If the model has not
   * been trained, and so coefficients
   * are null, the method computes the coefficients.
   * 
   * @param testData the test data to compute the RSME from
   * @return the RSME for the supplied test data
   */
  public double getTestDataRootMeanSquareError(List<Point> testData) {
    // model must be trained
    if (coefficients == null) {
      computeCoefficients();
    }

    double meanSquareError = testData.stream()
        .map(i -> Math.pow(getPrediction(i.getX()) - i.getY(), 2.0))
        .reduce(0.0, (a, b) -> a + b) / testData.size();

    return Math.sqrt(meanSquareError);
  }

  /**
   * Example used to find the optimal polynomial degree of a made-up function.
   * 
   * @param args the arguments required, in this case none.
   * @throws FileNotFoundException
   * @throws InterruptedException
   */
  public static void main(String[] args) throws FileNotFoundException, InterruptedException {
    File file = new File("src/testData2.txt");
    Scanner sc = new Scanner(file);
    sc.nextLine(); // ignores first line with comment
    List<Point> points = new ArrayList<>();
    List<Point> testData = new ArrayList<>();

    int count = 0;
    for (double i = -2.0; i < 47.0; i += 0.1) {
      // assume number of tokens is multiple of 2

      Point point = new Point(i, 0.7483924 * Math.pow(i, 7)
          + 13.431 * Math.pow(i, 6)
          + -12.35161212 * Math.pow(i, 5)
          + 0.0000012 * Math.pow(i, 4)
          + -9.99991212 * Math.pow(i, 3)
          + -34.4300009 * Math.pow(i, 2)
          + 0.7483924 * i
          + Math.random());

      if (count < 300) {
        points.add(point);
      } else {
        testData.add(point);
      }
      count++;
    }

    System.out.println("Points to analyse: " + testData.size());

    System.out.println("Optimal degree: " + getOptimalPolynomialDegreeWithTestData(points, testData));

  }

  // int count = 0;
  // for (double i = -2.0; i < 2.0; i += 0.01) {
  // //assume number of tokens is multiple of 2
  //
  // Point point = new Point(i, 0.7483924 * Math.pow(i, 7)
  // + 13.431 * Math.pow(i, 6)
  // + -12.35161212 * Math.pow(i, 5)
  // + 0.0000012 * Math.pow(i, 4)
  // + -9.99991212 * Math.pow(i, 3)
  // + -34.4300009 * Math.pow(i, 2)
  // + 0.7483924 * i
  // + Math.random());
  //
  // if (count < 200) {
  // points.add(point);
  // } else {
  // testData.add(point);
  // }
  // count++;
  // }

  /* EFFICIENCY ANALYSIS WITH DIFFERENT IMPLEMENTATIONS */
  // with this configuration:
  // naive inverse and naive normal equations: 129.705
  // qr inverse and naive normal equations: 79.82s
  // with qr decomposition and back substitution: 20.8814s
  // with qr decomposition, back substitution and distribution amongst threads:
  // 14.1829s

}
