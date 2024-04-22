#include "masterProblem.h"

void masterProblem::setDecisionsStageI()
{
	vector<Vessel> vesselSet = Data.getVesselSet();
	int Np = Data.getNumShipPath();
	     
	V = vector<vector<GRBVar>>(vesselSet.size(), vector<GRBVar>(Np));
	for (size_t h = 0; h < V.size(); h++)
	{
		for (size_t p = 0; p < Np; p++)
		{
			string VarName = "V[" + to_string(h) + "][" + to_string(p) + "]";
			V[h][p] = model.addVar(0, 1, vesselSet[h].GetOperationCost(), GRB_BINARY, VarName);
		}
	}

	size_t numPort = Data.getPortMap().size();
	L = vector<GRBVar>(numPort);
	for (size_t p = 0; p < numPort; p++)
	{
		string VarName = "L[" + to_string(p) + "]";
		L[p] = model.addVar(1000, 1000, 250, GRB_CONTINUOUS, VarName);
	}
}

void masterProblem::setDualDecisionsTypeI()
{
	vector<Request> RequestSet = Data.GetRequestSet();
	vector<int> Lodt1 = Data.getLodt1();
	vector<int> Lodt2 = Data.getLodt2();
	vector<int> Uodt1 = Data.getUodt1();
	vector<int> Uodt2 = Data.getUodt2();

	yita = model.addVar(-GRB_INFINITY, GRB_INFINITY, 1, GRB_CONTINUOUS, "yita");

	alpha1 = vector<GRBVar>(RequestSet.size());
	alpha2 = vector<GRBVar>(RequestSet.size());
	beta1 = vector<GRBVar>(RequestSet.size());
	beta2 = vector<GRBVar>(RequestSet.size());

	for (size_t od = 0; od < RequestSet.size(); od++)
	{
		alpha1[od] = model.addVar(0, M, Uodt1[od], GRB_CONTINUOUS, "alpha1[" + to_string(od) + "]");
		alpha2[od] = model.addVar(0, M, Uodt2[od], GRB_CONTINUOUS, "alpha2[" + to_string(od) + "]");
		beta1[od] = model.addVar(-M, 0, Lodt1[od], GRB_CONTINUOUS, "beta1[" + to_string(od) + "]");
		beta2[od] = model.addVar(-M, 0, Lodt2[od], GRB_CONTINUOUS, "beta2[" + to_string(od) + "]");
	}
}

void masterProblem::setDualDecisionsTypeII()
{
	vector<double> mean_request = Data.GetMeanRequest();
	vector<vector<double>> covariance_request = Data.GetCovarianceRequest();
	size_t num_request = mean_request.size();

	gamma = model.addVar(-M, M, 1, GRB_CONTINUOUS, "gamma");

	pi = vector<GRBVar>(num_request, GRBVar());
	Phi = vector<vector<GRBVar>>(num_request, vector<GRBVar>(num_request, GRBVar()));

	for (size_t i = 0; i < num_request; i++)
	{
		pi[i] = model.addVar(-M, M, mean_request[i], GRB_CONTINUOUS, "pi[" + to_string(i) + "]");

		for (size_t j = 0; j < num_request; ++j)
		{
			Phi[i][j] = model.addVar(-M, M, covariance_request[i][j], GRB_CONTINUOUS, "Phi[" + to_string(i) + "][" + to_string(j) + "]");
		}
	}
}

void masterProblem::setObjective()
{
	GRBLinExpr obj = 0;

	vector<Vessel> vesselSet = Data.getVesselSet();
	vector<Port> PortSet = Data.getPortSet();
	vector<int> num_rotation = Data.getNumRotation();

	// Stage I : item1 + item2
	// add item1
	for (size_t p = 0; p < Data.getNumShipPath(); p++)
	{
		for (size_t h = 0; h < vesselSet.size(); h++)
		{
			obj += vesselSet[h].GetOperationCost() * num_rotation[p] * V[h][p];
		}
	}

	//// add item2
	//for (size_t lp = 0; lp < PortSet.size(); lp++)
	//{
	//	obj += Data.getContainerPrice() * L[lp];
	//}

	// Stage II :
	// add item3
	if (SetType == Type1)
	{
		vector<int> Lodt1 = Data.getLodt1();
		vector<int> Lodt2 = Data.getLodt2();
		vector<int> Uodt1 = Data.getUodt1();
		vector<int> Uodt2 = Data.getUodt2();

		obj += yita;
		for (size_t od = 0; od < Data.GetRequestSet().size(); od++)
		{
			obj += alpha1[od] * Uodt1[od]
				+ alpha2[od] * Uodt2[od]
				+ beta1[od] * Lodt1[od]
				+ beta2[od] * Lodt2[od];
		}
	}
	else if (SetType == Type2)
	{
		vector<double> mean_request = Data.GetMeanRequest();
		vector<vector<double>> covariance_request = Data.GetCovarianceRequest();

		obj += gamma;

		for (size_t i = 0; i < mean_request.size(); ++i)
		{
			obj += mean_request[i] * pi[i];

			for (size_t j = 0; j < covariance_request.size(); ++j)
			{
				obj += covariance_request[i][j] * Phi[i][j];
			}
		}
	}

	model.setObjective(obj, GRB_MINIMIZE);
}

void masterProblem::setVesselConstraint1()
{
	vector<Vessel> vesselSet = Data.getVesselSet();

	for (size_t path = 0; path < Data.getNumShipPath(); path++)
	{
		GRBLinExpr left = 0;
		for (size_t h = 0; h < vesselSet.size(); h++)
		{
			left += V[h][path];
		}
		string name = "c-I-1[" + to_string(path) + "]";
		model.addConstr(left == 1, name);
	}
}

void masterProblem::setVesselConstraint2()
{
	vector<Vessel> vesselSet = Data.getVesselSet();

	for (size_t h = 0; h < vesselSet.size(); h++)
	{
		GRBLinExpr left = 0;
		for (size_t p = 0; p < Data.getNumShipPath(); p++)
		{
			left += V[h][p];
		}
		string name = "c-I-2[" + to_string(h) + "]";
		model.addConstr(left <= vesselSet[h].getMaxNum(), name);
	}
}

void masterProblem::addInitialCut()
{
	GRBLinExpr  left = yita;

	for (size_t od = 0; od < Data.GetRequestSet().size(); od++)
	{
		left += (alpha1[od] * Data.getUodt1()[od]);
		left += (alpha2[od] * Data.getUodt2()[od]);
		left += (beta1[od] * Data.getLodt1()[od]);
		left += (beta2[od] * Data.getLodt2()[od]);
	}

	model.addConstr(left >= 0);
}

void masterProblem::addOptimalityCut(vector<double>& demand, vector<double>& Uvalue, vector<double>& Vvalue, vector<double>& Wvalue)
{
	GRBLinExpr  right = 0;

	for (size_t od = 0; od < demand.size(); od++)
	{
		right += demand[od] * Uvalue[od];
	}

	for (size_t seg = 0; seg < Vvalue.size(); seg++)
	{
		for (size_t h = 0; h < Data.getVesselSet().size(); h++)
		{
			for (size_t r = 0; r < Data.getNumShipPath(); r++)
			{
				if (Data.getSegmentSet()[seg].first.getShipPathIndex() == r) {
					right += Vvalue[seg] * Data.getVesselSet()[h].GetCapacity() * V[h][r];
				}
			}
		}
	}

	for (size_t call = 0; call < Wvalue.size(); call++)
	{
		right += Wvalue[call] * L[Data.getCallNodeSet()[call].getPort().getPortIndex()];
	}

	for (size_t od = 0; od < demand.size(); od++)
	{
		right -= (alpha1[od] + beta1[od]) * demand[od] + (alpha2[od] + beta2[od]) * demand[od] * demand[od];
	}

	model.addConstr(yita >= right, "Cut" + to_string(optimalityCutNum));

	optimalityCutNum++;
}

void masterProblem::addFeasibilityCut(vector<double>& demand, vector<double>& Uvalue, vector<double>& Vvalue, vector<double>& Wvalue)
{
	cerr << "There is a error!" << endl;
}

void masterProblem::solveModel()
{
#ifndef DRO_PRINT_MP_LOG_
	model.set(GRB_IntParam_OutputFlag, 0);
#endif // !DRO_PRINT_MP_LOG_

	clock_t  start = clock();
	model.optimize();
	clock_t end = clock();

	// set (V,L)
	setVesselDecision();
	setInitialContainer();
	Data.calculateCapacity(VesselDecision);

	// set dual value
	setDualVarsValue();

	if (SetType == Type1) {
		model.write(filepath + "MP_model-I.lp");
	}
	else
	{
		model.write(filepath + "MP_model-II.lp");
	}

#ifdef DRO_PRINT_MP_SOLUTION_
	if (model.get(GRB_IntAttr_Status) == GRB_OPTIMAL)
	{
		cout << setprecision(2);
		cout << "Model Status : " << "Optimal" << endl;
		cout << "MP Optimal Obj = " << fixed << model.get(GRB_DoubleAttr_ObjVal) << endl;
		cout << "Model Solve Time = " << model.get(GRB_DoubleAttr_Runtime) << endl;
		printSolution();
	}
	else
	{
		// print violated constraint
		const int numConstrs = model.get(GRB_IntAttr_NumConstrs);
		auto constrs = model.getConstrs();
		for (int i = 0; i < numConstrs; ++i) {
			GRBConstr constr = constrs[i];
			double slack = constr.get(GRB_DoubleAttr_Slack);
			if (slack < 0) {
				double violation = -slack;
				cout << "Constraint " << constr.get(GRB_StringAttr_ConstrName) << " is violated by " << violation << endl;
			}
		}
	}
#endif // DRO_PRINT_MP_SOLUTION_
}

void masterProblem::printSolution()
{
	cout << "Vessel Ship Schedule : " << endl;
	for (size_t v = 0; v < Data.getNumShipPath(); v++)
	{
		cout << "Ship Path " << v << ": ";
		for (size_t h = 0; h < Data.getVesselSet().size(); h++)
		{
			int value_V_hv = static_cast<int>(model.getVarByName(V[h][v].get(GRB_StringAttr_VarName)).get(GRB_DoubleAttr_X) + 0.5);
			if (value_V_hv != 0)
			{
				cout << h << " ( " << Data.getVesselSet()[h].GetCapacity() << " TEUs )" << endl;
			}
		}
	}

	cout << "Initial Empty Containers £º" << endl;
	cout << "Port" << '\t' << "NumEmptyContainer" << endl;
	for (size_t p = 0; p < Data.getPortSet().size(); p++)
	{
		int L_value = model.getVarByName(L[p].get(GRB_StringAttr_VarName)).get(GRB_DoubleAttr_X);
		cout << Data.getPortSet()[p].getPort() << '\t' << L_value << endl;
	}

	cout << "Vessel Deployment Cost : " << vesselDeploymentCost << endl;
	cout << "Initial empty container Cost : " << initialContainerCost << endl;
	cout << "yita : " << yita.get(GRB_DoubleAttr_X) << endl;
}

void masterProblem::setVesselDecision()
{
	vesselDeploymentCost = 0;
	VesselDecision = vector<vector<int>>(Data.getVesselSet().size(), vector<int>(Data.getNumShipPath()));
	for (size_t h = 0; h < Data.getVesselSet().size(); h++)
	{
		for (size_t v = 0; v < Data.getNumShipPath(); v++)
		{
			VesselDecision[h][v] = int(V[h][v].get(GRB_DoubleAttr_X) + 0.5); 
			if (VesselDecision[h][v] == 1) {
				vesselDeploymentCost += Data.getVesselSet()[h].GetOperationCost() * Data.getNumRotation()[v];
			}
		}
	}
}

void masterProblem::setInitialContainer()
{
	initialContainerCost = 0;
	InitialContainer = vector<int>(Data.getPortSet().size());
	for (size_t p = 0; p < Data.getPortSet().size(); p++)
	{
		InitialContainer[p] = int(L[p].get(GRB_DoubleAttr_X));
		initialContainerCost = InitialContainer[p] * Data.getContainerPrice();
	}
}

void masterProblem::setDualVarsValue()
{
	alpha1Value = vector<double>(Data.GetRequestSet().size());
	alpha2Value = vector<double>(Data.GetRequestSet().size());
	beta1Value = vector<double>(Data.GetRequestSet().size());
	beta2Value = vector<double>(Data.GetRequestSet().size());

	for (size_t od = 0; od < Data.GetRequestSet().size(); od++)
	{
		alpha1Value[od] = alpha1[od].get(GRB_DoubleAttr_X);
		alpha2Value[od] = alpha2[od].get(GRB_DoubleAttr_X);
		beta1Value[od] = beta1[od].get(GRB_DoubleAttr_X);
		beta2Value[od] = beta2[od].get(GRB_DoubleAttr_X);
	}
}
