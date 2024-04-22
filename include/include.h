/*****************************************************************//**
 *  @file					: include.h
 *  @namespace		: DRO
 *  @brief				: include basic settings and common used head files:
 *								  basic settings: whether print solution and iteration log;
 *								  common head files: std, eigen, gurobi, and so on
 * 
 *  @author				: XuXw
 *  @version			: 1.0.0
 *  @date					: 2023/7/27
 *********************************************************************/

#ifndef _INCLUDE_H_
#define _INCLUDE_H_

#define DRO_USE_EIGEN_
//#define DRO_PRINT_DATA_

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <map>
#include <algorithm>
#include <utility>
#include <time.h>
#include <stdlib.h>
#include <random>

#include <eigen-3.3.8\Eigen\Dense>

#include "gurobi_c++.h"

#include "use_gurobi_win64_c++.h"

using namespace std;
using namespace Eigen;

enum AmbiguitySetType { Type1, Type2 };

/*
* print detailed solution
*/
//#define DRO_PRINT_SP_SOLUTION_H_
#define DRO_PRINT_DUAL_SP_SOLUTION_H_
#define DRO_PRINT_MP_SOLUTION_
//#define DRO_PRINT_MIP_DRO_SOLUTION_

/*
* print iteration log from solver
*/
//#define DRO_DETERMINE_OUTPUT_LOG_
//#define DRO_MIP_OUTPUT_LOG_
//#define DRO_SP_OUTPUT_LOG_
//#define DRO_DSP_OUTPUT_LOG_

#endif // !_INCLUDE_H_
