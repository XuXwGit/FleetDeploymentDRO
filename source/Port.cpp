#include "Port.h"

Port::Port()
{
}

Port::Port(int ID, string port, double loadingcost, double dischargecost,
	int turnovertime, int initialContainer)
	: port(port),
	portID(ID),
	turnoverTime(turnovertime),
	initialEmptyContainer(initialContainer),
	unitLoadingCost(loadingcost),
	unitDischargeCost(dischargecost)
{
}

Port::Port(const string& port, int portID, int portIndex, int turnoverTime, int initialEmptyContainer, int region, double unitLoadingCost, double unitDischargeCost)
	: port(port), portID(portID), portIndex(portIndex), turnoverTime(turnoverTime), initialEmptyContainer(initialEmptyContainer), region(region), unitLoadingCost(unitLoadingCost), unitDischargeCost(unitDischargeCost)
{
}
