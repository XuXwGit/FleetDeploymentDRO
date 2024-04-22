/*****************************************************************//**
 *  @file					: determine_model.h
 *  @namespace		: DRO
 *  @brief				: a deterministic model for two-stage problem (demand is mean demand)
 * 
 *  @author				: XuXw
 *  @version			: 1.0.0
 *  @date					: 2023/7/27
 *********************************************************************/

#ifndef DRO_DETERMINE_MODEL_
#define DRO_DETERMINE_MODEL_

#include "include.h"
#include "DroData.h"

class DetermineModel
{
public:
	DetermineModel(DroData& Data)
		: Data(Data)
	{
		filepath = Data.getFilepath();
		solveProblem();
	}

private:
	GRBEnv env = GRBEnv();
	GRBModel model = GRBModel(env);

	// vessel decision vars
	// V[h][r]
	vector<vector<GRBVar>> V;
	vector<GRBVar> L;

	// transport decision vars
	// X[od]
	// Y[od]
	// Z[od]
	// G[od]
	vector<GRBVar> X;
	vector<GRBVar> Y;
	vector<GRBVar> Z;
	vector<GRBVar> G;

	int M = 9999999;

	DroData Data;

	// output file path
	string filepath;

	// vessel decision solution
	vector<vector<int>> VesselDecision;
	vector<int> InitialContainer;

	double vesselDeploymentCost;
	double initialContainerCost;

public:

	// import data
	inline void setData(DroData& Data) { 
		this->Data = Data; 
	}

private:

	// add decision variables
	inline void addDecisionVars()
	{
		// set V_hr
		setDecisionsStageI();
		// set X,Y,Z,G
		setDecisionsStageII();
	}

	/*
	add decisions in Stage I, Stage II
	*/
	void setDecisionsStageI();
	void setDecisionsStageII();

	// set objective
	void setObjective();

	/*
	the following functions is about add constraints
	*/
	// add constraints
	inline void addConstraints()
	{
		setConstraintStageI();
		setConstraintStageII();
	}
	inline void setConstraintStageI() {
		setVesselConstraint();
	}
	inline void setConstraintStageII() {
		setDemandCosntraint();
		setVesselCapacityConstraint();
		setEmptyFlowBalanceConstraint();
	}

	// constraints in stage I : 
	// add vessel rotation constriant
	void setVesselConstraint();

	// constraints in stage II :
	// add demand constraint, i.e., at time t: demand(od) = X(od) + Z(od) + G(od)
	void setDemandCosntraint();
	// add vessel capacity constraint
	void setVesselCapacityConstraint();
	// add empty container flow balance, i.e., L_m^t >= 0 at any time t in any port p
	// in which L_m^t = sum(Flow in) - sum(Flow out)
	void setEmptyFlowBalanceConstraint();

	// solution
	// set Vessel decision
	void setVesselDecision();
	// set Initial Containers
	void setInitialContainer();

public:
	// solve problem
	inline void solveProblem() {
		addDecisionVars();
		setObjective();
		addConstraints();
		solveModel();
		printSolution();
	}
	inline void solveModel()
	{
#ifndef DRO_DETERMINE_OUTPUT_LOG_
		model.set(GRB_IntParam_OutputFlag, 0);
#endif DRO_DETERMINE_OUTPUT_LOG_
		clock_t  start = clock();
		model.optimize();
		clock_t end = clock();

		setVesselDecision();
		setInitialContainer();
		Data.calculateCapacity(VesselDecision);

		cout << "optimize time = " << (end - start) / CLOCKS_PER_SEC << endl;
		model.write(filepath + "model.lp");
		if (model.get(GRB_IntAttr_Status) == GRB_OPTIMAL)
		{
			cout << setprecision(2);
			cout << "Optimal Obj = " << fixed << model.get(GRB_DoubleAttr_ObjVal) << endl;
			cout << "Model Solve Time = " << model.get(GRB_DoubleAttr_Runtime) << endl;
		}
	}

	// get Vessel Decision
	inline vector<vector<int>>& getVesselDecision() {
		return VesselDecision;
	}

	inline vector<int>& getCapacity() {
		return Data.GetCapacity();
	}

	// get Initial Container
	inline vector<int>& getInitialContainer() {
		return InitialContainer;
	}

	// print solution
	void printSolution();
};

#endif // !_DETERMINE_MODEL_
