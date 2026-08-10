#pragma once
class Test
{
private:
	static bool isClose(PositionVector&, PositionVector&);
	static bool isClose(Matrix&, Matrix&);
public:
	static void createPositoinVector();
	static void createMatrix();
	static void createMatrix2();
	static void addMatrices();
	static void mulToMatrix();
	static void solveTest();
	static void dotTest();
	static void crossTest();
	static void outerTest();
	static void skewTest();
	static void LUTest();
	static void matrixChangeTest();
	static void matrixInputTest();
	static void mulTest();
	static void quat2mat2quat();
	static void allMatricesTests();
	
	static void quaternionTest();

	static void swapPositionVectorsTest();

	static void quatVectorMulTEst();

	static void controlDefactorTest();

	static void quatDivisionTest();

	static void eopTest();

	static void timeSubtractTest();

	static void magneticFieldMap();
	static void gravityMap();
	static void compareHolmesBelikov();

	static void satelliteChangesTest();
	static void satelliteGyrostatsTest();
	static void overlapModesTest();

	static void orbitIntegratorTest1();
	static void orbitIntegratorForcelessTest2();

	static void attitudeIntegratorTest();
	static void autostepTest();
	
	static void counterrotationTest();

	static void fullMotionIntegratorTest();

	static void rotationMatricesTest();

	static void inputFileTest();

	static void fullLaunchTest();

	static void solarPresureTest();

	static void multipleAssignmentTest();

	static void inputTest();
	static void inputParametersTest();
	static void inputJsonTest();

	static void gravityCoeffsTest();
	static void pnmTest();

	static void reactionWheelsTest();

	static void thrustersTest();

	static void solarPanelsTest();

	static void vtkSolarPressure();

	static void getSolarCoordinatesTest();

	static void fingMmZeroSolarPressureAttitude();

	static void checkNan();

	static void srplibTest();
};