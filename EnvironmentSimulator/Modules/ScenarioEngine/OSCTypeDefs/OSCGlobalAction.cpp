/*
 * esmini - Environment Simulator Minimalistic
 * https://github.com/esmini/esmini
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright (c) partners of Simulation Scenarios
 * https://sites.google.com/view/simulationscenarios
 */

#include "OSCGlobalAction.hpp"
#include "OSCSwarmTrafficGeometry.hpp"

using namespace scenarioengine;
using namespace STGeometry;

void ParameterSetAction::Start()
{
	LOG("Set parameter %s = %s", name_.c_str(), value_.c_str());
	parameters_->setParameterValue(name_, value_);
	OSCAction::Start();
}

void ParameterSetAction::Step(double dt, double simTime)
{
	OSCAction::Stop();
}

void SwarmTrafficAction::Start()
{
	LOG("SwarmTrafficAction Start");
	printf("IR: %f, SMjA: %f, SMnA: %f\n", innerRadius_, semiMajorAxis_, semiMinorAxis_);

	odrManager_ = roadmanager::Position::GetOpenDrive();

    initRoadSegments();
	OSCAction::Start();
}

void print_sols(char sols, double x1, double y1, double x2, double y2, bool polar) {
	switch(sols) {
        case 2: {
		    printf("Polar: %d, P1 = (%.2f, %.2f), P2 = (%.2f, %.2f)\n", polar, x1, y1, x2, y2);
		    break;
	    }
	    case 1: {
		    printf("Polar: %d, P1 = (%.2f, %.2f)\n", polar, x1, y1);
		    break;
	    }
	    default: {
		    printf("No point detected\n");
			break;
	    }
	}
}

void SwarmTrafficAction::Step(double dt, double simTime)
{
	LOG("SwarmTrafficAction Step");
	
	double x1, x2, y1, y2, s1, s2;

	roadmanager::Road* road = odrManager_->GetRoadByIdx(0);
	roadmanager::Geometry *geometry = road->GetGeometry(0);

	char sols = lineIntersect(centralObject_->pos_, static_cast<roadmanager::Line*>(geometry), semiMajorAxis_, semiMinorAxis_, &x1, &y1, &x2, &y2);
	print_sols(sols, x1, y1, x2, y2, false);

	//sols = polarLineIntersect(centralObject_->pos_, static_cast<roadmanager::Line*>(geometry), semiMajorAxis_, semiMinorAxis_, &s1, &s1);

	//printf("S: (%.2f, %.2f)\n", s1, s2);
	//switch(sols) {
    //    case 2: {
	//	    polar2cartesian(s1, geometry->GetX(), geometry->GetY(), geometry->GetHdg(), &x1, &y1);
	//		polar2cartesian(s2, geometry->GetX(), geometry->GetY(), geometry->GetHdg(), &x2, &y2);
	//	    break;
	//    }
	//    case 1: {
	//	    polar2cartesian(s1, geometry->GetX(), geometry->GetY(), geometry->GetHdg(), &x1, &y1);
	//	    break;
	//    }
	//}

	//sols = checkRange(static_cast<roadmanager::Line*>(geometry), sols, &x1, &x2, &y1, &y2);
	//print_sols(sols, x1, y1, x2, y2, true);

    if (++i == 1) { 
	    Vehicle* vehicle = new Vehicle();
	    vehicle->pos_.SetInertiaPos(x1, y2, centralObject_->pos_.GetH(), true);
		vehicle->controller_ = 0;
	    entities_.addObject(vehicle);
	}
	
}

void SwarmTrafficAction::initRoadSegments() {
	int roadId = centralObject_->pos_.GetTrackId();
	/* So far we have only one road, but in case of many, the intersection
	 * with the ellipses may happen between two distinct roads
	 */
    //front_.road = tail_.road = odrManager_->GetRoadById(roadId);

	LOG("Road segments initialised correctly");
}

// Get handle to road network
	//roadmanager::OpenDrive* odrManager = roadmanager::Position::GetOpenDrive();
	//for (size_t i = 0; i < odrManager->GetNumOfRoads(); i++)
	//{
	//	roadmanager::Road* road = odrManager->GetRoadByIdx((int)i);
	//	printf("Road %d length: %.2f\n", (int)i, road->GetLength());
	//	for (size_t j = 0; j < road->GetNumberOfGeometries(); j++)
	//	{
    //       roadmanager::Geometry* geometry = road->GetGeometry(static_cast<int>(j));
    //       printf("Road[%d], Geometry[%d]: s->%f, x->%f, y->%f, hdg->%f, l->%f\n", (int)i, (int)j, 
	//	   geometry->GetS(), 
	//	   geometry->GetX(), 
	//	   geometry->GetY(),
	//	   geometry->GetHdg(),
	//	   geometry->GetLength());
	//	}		
	//}
//printf("Central object world pos (x, y, hdg): %.2f, %.2f, %.2f\n", centralObject_->pos_.GetX(), centralObject_->pos_.GetY(), centralObject_->pos_.GetH());
	// printf("Central object road pos (roadId, s): %d, %.2f\n", centralObject_->pos_.GetTrackId(), centralObject_->pos_.GetS());