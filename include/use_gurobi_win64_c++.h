/*****************************************************************//**
 *  @file					: use_gurobi_win64_c++.h
 *  @namespace		: Solver
 *  @brief				: the basic setting for and visual studio and gurobi solver
 * 
 *  @author				: XuXw
 *  @version			: 1.0.0
 *  @date					: 2023/7/27
 *********************************************************************/

#ifndef C__gurobi100_win64_lib
#define C__gurobi100_win64_lib

#include "C:\\gurobi1000\\win64\\include\\gurobi_c++.h"
#include "C:\\gurobi1000\\win64\\include\\gurobi_c.h"

#pragma comment(lib,"C:\\gurobi1000\\win64\\lib\\gurobi100.lib")

#ifdef _DEBUG

#ifdef _DLL
#pragma comment(lib,"C:\\gurobi1000\\win64\\lib\\gurobi_c++mdd2017.lib")
#else
#pragma comment(lib,"C:\\gurobi1000\\win64\\lib\\gurobi_c++mtd2017.lib")
#endif // _DLL

#else

#ifdef _DLL
#pragma comment(lib,"C:\\gurobi1000\\win64\\lib\\gurobi_c++md2017.lib")
#else
#pragma comment(lib,"C:\\gurobi1000\\win64\\lib\\gurobi_c++mt2017.lib")
#endif // _DLL

#endif //DEBUG

#endif //C__gurobi100_win64_lib