/*****************************************************************//**
 *  @file					: route.h
 *  @namespace		: DRO
 *  @brief				: the shipping route (not "shipping path")
 * 
 *  @author				: XuXw
 *  @version			: 1.0.0
 *  @date					: 2023/7/28
 *********************************************************************/

#ifndef _ROUTE_H_
#define _ROUTE_H_

#include "include.h"
#include "port.h"

class Route
{
public:
	Route()
	{
	}

	inline Route(int routeID, int roundTime, vector<Port>& portSe, vector<int>& arrivalTime)
		: routeID(routeID),
		roundTripTime(roundTime),
		portSequence(portSe),
		arrivalTimeSequence(arrivalTime)
	{
	}

private:
	int routeID;
	int roundTripTime;
	vector<int> arrivalTimeSequence;
	vector<Port> portSequence;
public:
	inline int getRouteID() {
		return routeID;
	}
	inline int getRoundTripTime() { 
		return roundTripTime; 
	}
	inline vector<int>& getArrivalTimeSequence() {
		return arrivalTimeSequence;
	}
	inline vector<Port>& getPortSequence() {
		return portSequence;
	}
	inline void setPortSequence(vector<Port>& newPortSequence) {
		this->portSequence = newPortSequence;
	}
	inline void showRoute() {
		std::cout << routeID << "(" << roundTripTime << ") : ";
		for (size_t i = 0; i < portSequence.size(); i++)
		{
			if (i) {
				cout << "->";
			}
			std::cout << portSequence[i].getPort() << "(" << (i+1) << ", " << arrivalTimeSequence[i] << ") ";
		}
		std::cout << endl;
	}
};

#endif // !_ROUTE_H_
