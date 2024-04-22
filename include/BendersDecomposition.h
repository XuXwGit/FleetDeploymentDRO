/*****************************************************************//**
 *  @file					: BendersDecomposition.h
 *  @namespace		: DRO
 *  @brief				: Benders decomposition algorithm to solve two stage MIP-DRO
 * 
 *  @author				: XuXw
 *  @version			: 1.0.0
 *  @date					: 2023/7/27
 *********************************************************************/

#ifndef DRO_BENDERS_DECOMPOSITION_H_
#define DRO_BENDERS_DECOMPOSITION_H_

#include "include.h"
#include "DroData.h"
#include "masterProblem.h"
#include "dualSubproblem.h"

class BendersDecomposition
{
public:
	BendersDecomposition(DroData& data)
		: Data(data),
		MP(masterProblem(data)),
		DSP(DualSP(data))
	{
		frame();
	}

private:
	DroData Data;
	masterProblem MP;
	DualSP DSP;

private:
	void frame();
};

#endif // !DRO_BENDERS_DECOMPOSITION_H_
