#include "subproblem.h"

void SubProblem::addDecisionVars()
{
	vector<vector<OD>> Wod = Data.getWod();
	vector<Request> RequestSet = Data.GetRequestSet();
	size_t numRequest = RequestSet.size();
	size_t numPort = Data.getPortMap().size();

	X = vector<GRBVar>(numRequest, GRBVar());
	Y = vector<GRBVar>(numRequest, GRBVar());
	Z = vector<GRBVar>(numRequest, GRBVar());
	G = vector<GRBVar>(numRequest, GRBVar());

	for (size_t od = 0; od < numRequest; od++)
	{
		Port oo = RequestSet[od].getOrigin().getPort();
		Port dd = RequestSet[od].getDestination().getPort();

		string VarName = "X[" + to_string(od) + "]";
		X[od] = model.addVar(0, demand[od], 0, GRB_CONTINUOUS, VarName);
		VarName = "Y[" + to_string(od) + "]";
		Y[od] = model.addVar(0, demand[od], oo.getUnitLoadingCost() + dd.getUnitDischargeCost(), GRB_CONTINUOUS, VarName);

		double unitRental = Wod[oo.getPortIndex()][dd.getPortIndex()].GetUnitRentalCost();
		double unitPenalty = Wod[oo.getPortIndex()][dd.getPortIndex()].GetUnitPenaltyCost();

		VarName = "Z[" + to_string(od) + "]";
		Z[od] = model.addVar(0, demand[od], unitRental, GRB_CONTINUOUS, VarName);
		VarName = "G[" + to_string(od) + "]";
		G[od] = model.addVar(0, demand[od], unitPenalty, GRB_CONTINUOUS, VarName);
	}
}

void SubProblem::setObjective()
{
	GRBLinExpr obj = 0;
	vector<Vessel> vesselSet = Data.getVesselSet();
	vector<vector<OD>> Wod = Data.getWod();
	vector<Request> requestSet = Data.GetRequestSet();
	vector<Port> PortSet = Data.getPortSet();
	size_t numPort = Data.getPortMap().size();
	size_t numRequest = requestSet.size();

	int od = 0;
	for (size_t od = 0; od < numRequest; od++)
	{
		Port oo = requestSet[od].getOrigin().getPort();
		Port dd = requestSet[od].getDestination().getPort();

		double unitRental = Wod[oo.getPortIndex()][dd.getPortIndex()].GetUnitRentalCost();
		double unitPenalty = Wod[oo.getPortIndex()][dd.getPortIndex()].GetUnitPenaltyCost();

		obj += (oo.getUnitLoadingCost() + dd.getUnitDischargeCost()) * Y[od] + unitRental * Z[od] + unitPenalty * G[od];
	}
	model.setObjective(obj, GRB_MINIMIZE);
}

void SubProblem::setDemandCosntraint()
{
	vector<Request> RequestSet = Data.GetRequestSet();
	size_t numRequest = RequestSet.size();
	DemandConstrSet = vector<GRBConstr>(numRequest);

	string name;
	for (size_t od = 0; od < numRequest; od++)
	{
		name = "c-II-2[" + to_string(od) + "]";
		DemandConstrSet[od] = model.addConstr(X[od] + Z[od] + G[od] == demand[od], name);
	}
}

void SubProblem::setVesselCapacityConstraint()
{
	typedef pair<CallNode, CallNode> SegMent;
	vector<SegMent> segmentSet = Data.getSegmentSet();
	vector<Request> requestSet = Data.GetRequestSet();
	vector<vector<int>> transitTime = Data.getTransitTime();
	vector<int> portCallSet = Data.getPortCallSet();
	vector<Vessel> vesselSet = Data.getVesselSet();
	map<string, Port> M = Data.getPortMap();
	size_t numSegment = segmentSet.size();

	CapacityConstrSet = vector<GRBConstr>(numSegment);

	string name;
	for (size_t seg = 0; seg < numSegment; seg++)
	{
		int p = segmentSet[seg].first.getShipPathIndex();

		double originTime = segmentSet[seg].first.getArrivalTime();
		double destinationTime = segmentSet[seg].second.getArrivalTime();

		GRBLinExpr left = 0;

		for (size_t od = 0; od < requestSet.size(); od++)
		{
			if (requestSet[od].getOrigin().getShipPathIndex() != p)
				continue;
			if (requestSet[od].getOrigin().getArrivalTime() <= originTime
				&& requestSet[od].getDestination().getArrivalTime() >= destinationTime)
			{
				left += X[od] + Y[od] + Z[od];
			}
		}

		GRBLinExpr right = 0;
		for (size_t h = 0; h < vesselSet.size(); h++)
		{
			right += vesselSet[h].GetCapacity() * VesselDecision[h][p];
		}

		string name = "c-II-1[" + to_string(seg) + "]";
		CapacityConstrSet[seg] = model.addConstr(left <= right, name);
	}
}

void SubProblem::setEmptyFlowBalanceConstraint()
{
	// M - port_set
	map<string, Port> M = Data.getPortMap();
	vector<vector<OD>> Wod = Data.getWod();
	vector<vector<int>> transitTime = Data.getTransitTime();
	vector<vector<bool>> callWithPort = Data.GetThetaMp();
	vector<CallNode> callNodeSet = Data.getCallNodeSet();
	vector<Request> requestSet = Data.GetRequestSet();

	size_t numCallNode = callNodeSet.size();
	FlowBalanceConstrSet = vector<GRBConstr>(numCallNode);

	string constrName = "C-II-3";
	for (size_t n = 0; n < numCallNode; n++)
	{
		CallNode np = callNodeSet[n];

		GRBLinExpr left = 0;

		for (size_t od = 0; od < requestSet.size(); od++)
		{
			CallNode origin = requestSet[od].getOrigin();
			CallNode destination = requestSet[od].getDestination();
			CallNode release = requestSet[od].getReleaseNode();

			bool flag = requestSet[od].getWhetherEmptyContainer();

			// empty container -> laden container (transport to other port)
			if (origin.getArrivalTime() <= np.getArrivalTime() && origin.getPort() == np.getPort())
			{
				left += X[od];
			}

			// empty container -> empty container (reposition to other port)
			if (origin.getArrivalTime() <= np.getArrivalTime() && origin.getPort() == np.getPort())
			{
				left += Y[od];
			}

			// laden container -> empty container (release after arrival and turnover)
			if (flag && release.getArrivalTime() <= np.getArrivalTime() && release.getPort() == np.getPort())
			{
				left -= X[od];
			}

			// empty container -> empty container (reposition from other port to this port)
			if (destination.getArrivalTime() <= np.getArrivalTime() && destination.getPort() == np.getPort())
			{
				left -= Y[od];
			}
		}

		FlowBalanceConstrSet[n] = model.addConstr(left <= InitialContainer[Data.getCallNodeSet()[n].getPort().getPortIndex()], constrName + "[" + to_string(n) + "]");
	}
}

void SubProblem::printSolution()
{
	if (model.get(GRB_IntAttr_Status) == GRB_OPTIMAL)
	{
		cout << "Request Solution :" << endl;
		cout << "demand" << "\t\t" << "Self-Own" << "\t\t" << "Leased" << "\t\t" << "Unfullfill" << endl;
		for (size_t re = 0; re < Data.GetMeanDemand().size(); re++)
		{
			int X_value = model.getVarByName(X[re].get(GRB_StringAttr_VarName)).get(GRB_DoubleAttr_X);
			int Z_value = model.getVarByName(Z[re].get(GRB_StringAttr_VarName)).get(GRB_DoubleAttr_X);
			int G_value = model.getVarByName(G[re].get(GRB_StringAttr_VarName)).get(GRB_DoubleAttr_X);

			cout << Data.GetRequestSet()[re].getOrigin().getPort().GetRegion() << "->"
				<< Data.GetRequestSet()[re].getDestination().getPort().GetRegion() << "\t"
				<< Data.GetMeanDemand()[re] << "\t\t"
				<< X_value << "\t\t" << Z_value << "\t\t"
				<< G_value << endl;
		}
	}
	else
	{
		cout << "!! Not Optimal !!" << endl;
	}
}
