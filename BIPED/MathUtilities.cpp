#include "../include/math/MathUtilities.h"

Eigen::MatrixXf Matrix_Pow(Eigen::MatrixXf Mat, size_t n)
{
	//MatrixXd RES=MatrixX4d::Identity();
	Eigen::MatrixXf RES;

	RES = Eigen::MatrixXf::Identity(Mat.rows(), Mat.cols());
	while (n != 0)
	{
		if (n & 1)
			RES *= Mat;
		Mat *= Mat;
		n >>= 1;
	}
	return RES;
}
