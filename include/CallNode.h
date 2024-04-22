/*****************************************************************//**
 *  @file					: CallNode.h
 *  @namespace		: DRO
 *  @brief				: a class for "port call" node , which include : "Port", "arrivalTime", "shipPathIndex", "shipRotation";
 * 
 *  @author				: XuXw
 *  @version			: 1.0.0
 *  @date					: 2023/7/28
 *********************************************************************/

#ifndef FDP_DRO_CALL_NODE_H_
#define FDP_DRO_CALL_NODE_H_

#include "include.h"
#include "port.h"

class CallNode
{
public:

private:
	Port port;
	int arrivalTime;
	int shipPathIndex;
	int shipRotation;
public:

	CallNode(const Port& port, int arrivalTime, int shipPath, int shipRotation)
		: port(port), arrivalTime(arrivalTime), shipPathIndex(shipPath), shipRotation(shipRotation)
	{
	}

	inline Port& getPort() {
		return port;
	}
	inline string get_port() {
		return port.getPort();
	}
	inline int getArrivalTime() {
		return arrivalTime;
	}
	inline int getShipPathIndex() {
		return shipPathIndex;
	}
	inline void setPort(const Port& port) {
		this->port = port;
	}
};

#endif // !FDP_DRO_CALL_NODE_H_

