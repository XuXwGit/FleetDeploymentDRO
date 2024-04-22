/*****************************************************************//**
 *  @file					: masterProblem.h
 *  @namespace		: DRO
 *  @brief				: the master problem of MIP-DRO 
 * 
 *  @author				: XuXw
 *  @version			: 1.0.0
 *  @date					: 2023/7/27
 *********************************************************************/

#ifndef DRO_MASTER_PROBLEM_H_
#define DRO_MASTER_PROBLEM_H_

#include "include.h"
#include "DroData.h"

class masterProblem
{
public:
	masterProblem(){}
	masterProblem(DroData& data)
		: Data(data)
	{}
private:
	GRBEnv env = GRBEnv();
	GRBModel model = GRBModel(env);

	// input
	// Data / Parameter
	DroData Data;
	int M = 99999999;
	// uncertain set
	AmbiguitySetType SetType = Type1;

	// output
	string filepath;

	// decision variables

	// Stage I 
	// vessel decision vars
	// V[h][t]
	vector<vector<GRBVar>> V;
	// initial empty containers
	// L[p]
	vector<GRBVar> L;

	// dual variables for type1 ambiguity set
	// yita
	GRBVar yita;

	// alpha1[od]
	// alpha2[od]
	vector<GRBVar> alpha1;
	vector<GRBVar> alpha2;
	// beta1[od]
	// beta2[od]
	vector<GRBVar> beta1;
	vector<GRBVar> beta2;

	// dual variables fot type2 ambiguity set
	// gamma
	GRBVar gamma;
	// pi[i]
	vector<GRBVar> pi;
	// Phi[i][i'] for type-II
	vector<vector<GRBVar>> Phi;

	// vessel decision solution
	vector<vector<int>> VesselDecision;
	vector<int> InitialContainer;

	double vesselDeploymentCost;
	double initialContainerCost;

	// dual decision value
	vector<double> alpha1Value;
	vector<double> alpha2Value;
	vector<double> beta1Value;
	vector<double> beta2Value;

	// int number of optimality cut
	int optimalityCutNum = 0;

private:
	// Decision variables
	inline void addDecisionVars()
	{
		// Type-I Ambiguity Set : moment-based first-moment and second- moment
		setDecisionsStageI();
		if (SetType == Type1)
		{
			setDualDecisionsTypeI();
		}
		// Type-I Ambiguity Set : moment-based first-moment and convariance matrix
		else if (SetType == Type2)
		{
			setDualDecisionsTypeII();
		}
	}

	/**
	 * add decisions for first stage : (V, L)
	*/
	void setDecisionsStageI();

	// set dual decisions for moment-based ambiguity set
	// type-I
	void setDualDecisionsTypeI();
	// type-II
	void setDualDecisionsTypeII();


	// Objective
	// set objective
	void setObjective();

	// Constraints
	// addConstraints
	inline void addConstraints()
	{
		setConstraintStageI();
	}

	/*
	the following functions is about add constraints
	*/
	inline void setConstraintStageI() {
		setVesselConstraint1();
		setVesselConstraint2();
	}

	// constraints in stage I : 
	// add vessel rotation constriant
	void setVesselConstraint1();
	// add vessel number constraint
	void setVesselConstraint2();

	// solution
	// set Vessel decision
	void setVesselDecision();
	// set Initial Containers
	void setInitialContainer();
	// set dual variables decisions
	void setDualVarsValue();

public:
	// add initial cut
	void addInitialCut();

	// add optimality cut
	void addOptimalityCut(vector<double>& demand, vector<double>& Uvalue, vector<double>& Vvalue, vector<double>& Wvalue);

	// add feasibility cut
	void addFeasibilityCut(vector<double>& demand, vector<double>& Uvalue, vector<double>& Vvalue, vector<double>& Wvalue);

public:
	//build model
	inline void buildModel() {
		addDecisionVars();
		setObjective();
		addConstraints();
		addInitialCut();
	}

	// solve DRO MIP model
	void solveModel();

	// print solution
	void printSolution();

	// get the vessel decision on each segment
	vector<int>& getCapacity() {
		return Data.GetCapacity();
	}

	// get initial container on each port
	vector<int>& getInitialContainer() {
		return InitialContainer;
	}

	// get solve status
	inline int getModelStatus() {
		return model.get(GRB_IntAttr_Status);
	}

	// get objective value
	inline double getObjective() {
		return model.get(GRB_DoubleAttr_ObjVal);
	}

	// get recource obj
	inline double getRecourceObj() {
		return yita.get(GRB_DoubleAttr_X);
	}

	// get first stage obj
	inline double getFirstStageObj() {
		return model.get(GRB_DoubleAttr_ObjVal) - getRecourceObj();
	}

	// sum moment : 
	// (alpha1 + beta1) * demand(omega) + (alpha2 + beta2) * demand(omega) ^2
	// calculate true moment sum distance ?
	// get sum moment
	inline double getSumMoment(vector<double>& demand) {
		double sum_moment = 0;

		for (size_t od = 0; od < demand.size(); od++)
		{
			sum_moment += (alpha1Value[od] + beta1Value[od]) * demand[od] 
				+ (alpha2Value[od] + beta2Value[od]) * demand[od] * demand[od];
		}

		return sum_moment;
	}
};


#endif // !DRO_MASTER_PROBLEM_H_
