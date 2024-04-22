#include "dualSubproblem.h"

void DualSP::resetObjective(vector<double>& demand, vector<int>& capacity, vector<int>& initialContainer)
{
	this->demand = demand;
	this->capacity = capacity;
	this->InitialContainer = initialContainer;
	obj = 0;

   	for (size_t od = 0; od < demand.size(); od++)
	{
		obj += demand[od] * U[od];
	}
	for (size_t seg = 0; seg < capacity.size(); seg++)
	{
		obj += capacity[seg] * V[seg];
	}
	for (size_t call = 0; call < W.size(); call++)
	{
		obj += InitialContainer[Data.getCallNodeSet()[call].getPort().getPortIndex()] * W[call];
	}
	model.setObjective(obj, GRB_MAXIMIZE);
}

void DualSP::addDecisionVars()
{
	size_t numRequest = Data.GetRequestSet().size();
	size_t numSegment = Data.getSegmentSet().size();
	size_t numCallNode = Data.getCallNodeSet().size();

	U = vector<GRBVar>(numRequest, GRBVar());
	V = vector<GRBVar>(numSegment, GRBVar());
	W = vector<GRBVar>(numCallNode, GRBVar());

	for (size_t od = 0; od < numRequest; od++)
	{
		U[od] = model.addVar(-GRB_INFINITY, GRB_INFINITY, demand[od], GRB_CONTINUOUS, "U[" + to_string(od)+"]");
	}

	for (size_t seg = 0; seg < numSegment; seg++)
	{
		V[seg] = model.addVar(-GRB_INFINITY, 0, capacity[seg], GRB_CONTINUOUS, "V[" + to_string(seg)+"]");
	}
	     
	for (size_t np = 0; np < numCallNode; np++)
	{
		W[np] = model.addVar(-GRB_INFINITY, 0, InitialContainer[Data.getCallNodeSet()[np].getPort().getPortIndex()], GRB_CONTINUOUS, "W[" + to_string(np) + "]");
	}
}

void DualSP::setObjective()
{
	for (size_t od = 0; od < Data.GetRequestSet().size(); od++)
	{
		obj += demand[od] * U[od];
	}
	for (size_t seg = 0; seg < Data.getSegmentSet().size(); seg++)
	{
		obj += capacity[seg] * V[seg];
	}
	for (size_t np = 0; np < Data.getCallNodeSet().size(); np++)
	{
		obj += InitialContainer[Data.getCallNodeSet()[np].getPort().getPortIndex()] * W[np];
	}
	model.setObjective(obj, GRB_MAXIMIZE);
}

void DualSP::setConstraintX()
{
	vector<Request> requestSet = Data.GetRequestSet();
	vector<DroData::SegMent> segmentSet = Data.getSegmentSet();
	vector<CallNode> callnodeSet = Data.getCallNodeSet();
	size_t numRequest = requestSet.size();
	size_t numSegment = segmentSet.size();
	size_t numCallnode = callnodeSet.size();

	ConstrX = vector<GRBConstr>(numRequest);

	for (size_t od = 0; od < numRequest; od++)
	{
		GRBLinExpr left = U[od];

		for (size_t seg = 0; seg < numSegment; seg++)
		{
			double originTime = segmentSet[seg].first.getArrivalTime();
			double destinationTime = segmentSet[seg].second.getArrivalTime();

			// if the request is not the same as the ship path of the segment, then skip
			if (requestSet[od].getOrigin().getShipPathIndex() != segmentSet[seg].first.getShipPathIndex())
				continue;
			if (requestSet[od].getOrigin().getArrivalTime() <= originTime
				&& requestSet[od].getDestination().getArrivalTime() >= destinationTime)
			{
				left += V[seg];
			}
		}

		for (size_t np = 0; np < callnodeSet.size(); np++)
		{
			CallNode origin = requestSet[od].getOrigin();
			CallNode destination = requestSet[od].getDestination();
			CallNode release = requestSet[od].getReleaseNode();

			bool flag = requestSet[od].getWhetherEmptyContainer();

			// laden container -> empty container (release after arrival and turnover)
			if (flag && release.getArrivalTime() <= callnodeSet[np].getArrivalTime() && release.getPort() == callnodeSet[np].getPort())
			{
				left -= W[np];
			}

			// empty container -> laden container (transport to other port)
			if (origin.getArrivalTime() <= callnodeSet[np].getArrivalTime() && origin.getPort() == callnodeSet[np].getPort())
			{
				left += W[np];
			}
		}

		ConstrX[od] = model.addConstr(left <= 0, "ConstrX[" + to_string(od) + "]");
	}
}

void DualSP::setConstraintY()
{
	vector<Request> requestSet = Data.GetRequestSet();
	vector<DroData::SegMent> segmentSet = Data.getSegmentSet();
	vector<CallNode> callnodeSet = Data.getCallNodeSet();
	vector<vector<OD>> Wod = Data.getWod();

	size_t numRequest = requestSet.size();
	size_t numSegment = segmentSet.size();
	size_t numCallnode = callnodeSet.size();

	ConstrY = vector<GRBConstr>(numRequest);

	for (size_t od = 0; od < numRequest; od++)
	{
		GRBLinExpr left = 0;
		for (size_t seg = 0; seg < numSegment; seg++)
		{
			double originTime = segmentSet[seg].first.getArrivalTime();
			double destinationTime = segmentSet[seg].second.getArrivalTime();

			// if the request is not the same as the ship path of the segment, then skip
			if (requestSet[od].getOrigin().getShipPathIndex() != segmentSet[seg].first.getShipPathIndex())
				continue;
			if (requestSet[od].getOrigin().getArrivalTime() <= originTime
				&& requestSet[od].getDestination().getArrivalTime() >= destinationTime)
			{
				left += V[seg];
			}
		}

		for (size_t np = 0; np < numCallnode ; np++)
		{
			CallNode origin = requestSet[od].getOrigin();
			CallNode destination = requestSet[od].getDestination();
			CallNode release = requestSet[od].getReleaseNode();

			bool flag = requestSet[od].getWhetherEmptyContainer();

			// empty container -> empty container (reposition from other port)
			if (destination.getArrivalTime() <= callnodeSet[np].getArrivalTime() && destination.getPort() == callnodeSet[np].getPort())
			{
				left -= W[np];
			}

			// empty container -> empty container (reposition to other port)
			if (origin.getArrivalTime() <= callnodeSet[np].getArrivalTime() && origin.getPort() == callnodeSet[np].getPort())
			{
				left += W[np];
			}
		}

		Port oo = requestSet[od].getOrigin().getPort();
		Port dd = requestSet[od].getDestination().getPort();

		ConstrY[od] = model.addConstr(left <= (oo.getUnitLoadingCost() + dd.getUnitDischargeCost()), "ConstrY[" + to_string(od) + "]");
	}
}

void DualSP::setConstraintZ()
{
	vector<Request> requestSet = Data.GetRequestSet();
	vector<DroData::SegMent> segmentSet = Data.getSegmentSet();
	vector<CallNode> callnodeSet = Data.getCallNodeSet();
	vector<vector<OD>> Wod = Data.getWod();

	size_t numRequest = requestSet.size();
	size_t numSegment = segmentSet.size();
	size_t numCallnode = callnodeSet.size();

	ConstrZ = vector<GRBConstr>(numRequest);

	for (size_t od = 0; od < numRequest; od++)
	{
		GRBLinExpr left = U[od];
		for (size_t seg = 0; seg < numSegment; seg++)
		{
			double originTime = segmentSet[seg].first.getArrivalTime();
			double destinationTime = segmentSet[seg].second.getArrivalTime();

			// if the request is not the same as the ship path of the segment, then skip
			if (requestSet[od].getOrigin().getShipPathIndex() != segmentSet[seg].first.getShipPathIndex())
				continue;
			if (requestSet[od].getOrigin().getArrivalTime() <= originTime
				&& requestSet[od].getDestination().getArrivalTime() >= destinationTime)
			{
				left += V[seg];
			}
		}

		Port oo = requestSet[od].getOrigin().getPort();
		Port dd = requestSet[od].getDestination().getPort();
		double unitRental = Wod[oo.getPortIndex()][dd.getPortIndex()].GetUnitRentalCost();

		ConstrZ[od] = model.addConstr(left <= unitRental, "ConstrZ[" + to_string(od) + "]");
	}
}

void DualSP::setConstraintG()
{
	vector<Request> requestSet = Data.GetRequestSet();
	vector<DroData::SegMent> segmentSet = Data.getSegmentSet();
	vector<CallNode> callnodeSet = Data.getCallNodeSet();
	vector<vector<OD>> Wod = Data.getWod();

	size_t numRequest = requestSet.size();
	size_t numSegment = segmentSet.size();
	size_t numCallnode = callnodeSet.size();

	ConstrG = vector<GRBConstr>(numRequest);

	for (size_t od = 0; od < numRequest; od++)
	{
		Port oo = requestSet[od].getOrigin().getPort();
		Port dd = requestSet[od].getDestination().getPort();

		double unitPenalty = Wod[oo.getPortIndex()][dd.getPortIndex()].GetUnitPenaltyCost();

		ConstrY[od] = model.addConstr(U[od] <= unitPenalty, "ConstrG[" + to_string(od) + "]");
	}
}

void DualSP::setUvalue()
{
	// initial Uvalue
	Uvalue = vector<double>(U.size());

	for (size_t od = 0; od < Uvalue.size(); od++)
	{
		Uvalue[od] = U[od].get(GRB_DoubleAttr_X);
	}
}

void DualSP::setVvalue()
{
	// initial Uvalue
	Vvalue = vector<double>(V.size());

	for (size_t a = 0; a < Vvalue.size(); a++)
	{
		Vvalue[a] = V[a].get(GRB_DoubleAttr_X);
	}
}

void DualSP::setWvalue()
{
	// initial Uvalue
	Wvalue = vector<double>(W.size());

	for (size_t n = 0; n < Wvalue.size(); n++)
	{
		Wvalue[n] = W[n].get(GRB_DoubleAttr_X);
	}
}
