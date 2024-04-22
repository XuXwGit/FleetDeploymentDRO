#include "BendersDecomposition.h"

void BendersDecomposition::frame()
{
	// build model for mp and dsp
	MP.buildModel();
	DSP.bulidModel();

	double UB = GRB_INFINITY;
	double LB = -GRB_INFINITY;

	double epsion = 0.01;

	int k = 0;

	while (UB - LB > epsion)
	{
		// solve master problem
		MP.solveModel();
		// update LB
		LB = LB > MP.getObjective() ? LB : MP.getObjective();

		// solve sub problem
		double max_dsp = -GRB_INFINITY;
		for (size_t s = 0; s < Data.getDemandSet().size(); s++)
		{
			DSP.resetObjective(Data.getDemandSet()[s], MP.getCapacity(), MP.getInitialContainer());
			DSP.solveModel();

			MP.addOptimalityCut(Data.getDemandSet()[s], DSP.getUvalue(), DSP.getVvalue(), DSP.getWvalue());

			//cout << DSP.getObjective() - MP.getSumMoment(Data.getDemandSet()[s])<<endl;
			//cout << MP.getObjective() - MP.getRecourceObj() << endl;
			//cout << MP.getObjective() - MP.getRecourceObj() + DSP.getObjective() - MP.getSumMoment(Data.getDemandSet()[s]) << endl;

			max_dsp = max_dsp > DSP.getObjective() - MP.getSumMoment(Data.getDemandSet()[s]) ? max_dsp :  DSP.getObjective() - MP.getSumMoment(Data.getDemandSet()[s]);
		}
		UB = UB < MP.getFirstStageObj() + max_dsp ? UB : MP.getFirstStageObj() + max_dsp;

		cout << k++ << "\t\t" << UB << "\t\t" << LB << endl;
	} 
}
