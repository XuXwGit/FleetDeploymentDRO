/*****************************************************************//**
 *  @file					: Port.h
 *  @namespace		: DRO
 *  @brief				: the "Port" class, include information / data about port (not port call)
 * 
 *  @author				: XuXw
 *  @version			: 1.0.0
 *  @date					: 2023/7/28
 *********************************************************************/

#ifndef _PORT_H_
#define _PORT_H_

#include "include.h"

class Port
{
public:
	Port();
	Port(int ID, string port, double loadingcost, double dischargecost, 
			 int turnovertime, int initialContainer);

	Port(const string& port, int portID, int portIndex, int turnoverTime, int initialEmptyContainer, int region, double unitLoadingCost, double unitDischargeCost);

	bool operator==(const Port& other) const
	{
		if (portID == other.portID) {
			return true;
		}
		return false;
	}

	// operator = 
	Port& operator=(const Port& other) {
		if (this == &other) {
			return *this;  
		}

		port = other.port;
		portID = other.portID;
		portIndex = other.portIndex;
		turnoverTime = other.turnoverTime;
		initialEmptyContainer = other.initialEmptyContainer;
		region = other.region;
		unitLoadingCost = other.unitLoadingCost;
		unitDischargeCost = other.unitDischargeCost;

		return *this;
	}

private:
	string port;
	int portID;
	int portIndex;
	int turnoverTime;
	int initialEmptyContainer;
	int region;
	double unitLoadingCost;
	double unitDischargeCost;

public:
	inline string getPort()
	{
		return port;
	}

	inline int getPortID()
	{
		return portID;
	}

	inline int getPortIndex()
	{
		return portIndex;
	}

	inline int getTurnoverTime()
	{
		return turnoverTime;
	}

	inline int getInitialEmptyContainer()
	{
		return initialEmptyContainer;
	}

	inline double getUnitLoadingCost()
	{
		return unitLoadingCost;
	}

	inline double getUnitDischargeCost()
	{
		return unitDischargeCost;
	}

	inline void setPort(string port)
	{
		this->port = port;
	}

	inline void setPortID(int portID)
	{
		this->portID = portID;
	}

	inline void setPortIndex(int portIndex)
	{
		this->portIndex = portIndex;
	}

	inline void setTurnoverTime(int turnoverTime)
	{
		this->turnoverTime = turnoverTime;
	}

	inline void setInitialEmptyContainer(int initialEmptyContainer)
	{
		this->initialEmptyContainer = initialEmptyContainer;
	}

	inline void setUnitLoadingCost(double unitLoadingCost)
	{
		this->unitLoadingCost = unitLoadingCost;
	}

	inline void setUnitDischargeCost(double unitDischargeCost)
	{
		this->unitDischargeCost = unitDischargeCost;
	}

	inline int GetRegion() const
	{
		return region;
	}

	inline void SetRegion(const int region)
	{
		this->region = region;
	}

	inline void ShowPort() const
	{
		std::cout << portID
			<< '\t' << port
			<< '\t' << region
			<< '\t' << turnoverTime
			<< '\t' << initialEmptyContainer
			<< '\t' << unitLoadingCost
			<< '\t' << unitDischargeCost
			<< endl;
	}

};

#endif // !_PORT_H_
