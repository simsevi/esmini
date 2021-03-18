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
	detectPoints();
	spawn(ellipse_.lower, centralObject_->pos_.GetLaneId(), 0);
	spawn(ellipse_.upper, 1, 1);
	despawn();
	
}

void SwarmTrafficAction::initRoadSegments() {

}

void SwarmTrafficAction::spawn(segmentInfo &segment, int lane, double hdg_offset) {
	if (segment.segmentIdx == -1) return;

	// Ensure some distance between two spawned veichles
	if (segment.last){
		roadmanager::Position pos, c_pos;
		c_pos = segment.last->pos_;
		pos.XYZH2TrackPos(segment.x, segment.y, 0, c_pos.GetH() + hdg_offset * M_PI);
		if (abs(pos.GetS() - c_pos.GetS()) < 20) return;
	}

	Vehicle* vehicle = new Vehicle();
	vehicle->pos_.SetInertiaPos(segment.x, segment.y, centralObject_->pos_.GetH() + hdg_offset * M_PI, true);
	vehicle->pos_.SetLanePos(vehicle->pos_.GetTrackId(), lane, vehicle->pos_.GetS(), 0);
	vehicle->SetSpeed(centralObject_->GetSpeed());
	vehicle->controller_     = 0;
	vehicle->model_filepath_ = centralObject_->model_filepath_;
	int id                   = entities_->addObject(vehicle);
	vehicle->name_           = std::to_string(id);
	segment.last             = vehicle;
	vehiclesId_.push_back(id);
}

bool SwarmTrafficAction::detectPoints() {
	double s1, s2;
	char sols;
	segmentInfo seg1, seg2;
	roadmanager::Position pos;

	roadmanager::Road* road = odrManager_->GetRoadByIdx(0);
    
	seg1.road = seg2.road = road;
	for (size_t i = 0; i < road->GetNumberOfGeometries(); i++) {
		roadmanager::Geometry *geometry = road->GetGeometry(i);
		sols = lineIntersect(centralObject_->pos_, 
		                     static_cast<roadmanager::Line*>(geometry), 
							 semiMajorAxis_, semiMinorAxis_, 
							 &seg1.x, &seg1.y, &seg2.x, &seg2.y);
		switch (sols) {
			case 2:
			    seg1.segmentIdx = seg2.segmentIdx = i;
			case 1:
			    seg1.segmentIdx = i;
		}
	}

	switch(sols) {
		case 2: { 
			printf("Detected 2 points\n");
		    pos.XYZH2TrackPos(seg1.x, seg1.y, 0, seg1.road->GetGeometry(seg1.segmentIdx)->GetHdg());
			s1 = pos.GetS();

			pos.XYZH2TrackPos(seg2.x, seg2.y, 0, seg2.road->GetGeometry(seg2.segmentIdx)->GetHdg());
			s2 = pos.GetS();

			if (s1 <= s2) {
				ellipse_.lower = seg1;
				ellipse_.upper = seg2;
			} else {
				ellipse_.lower = seg2;
				ellipse_.upper = seg1;
			}
			break;
		}
		case 1: { 
			printf("Detected 1 points\n");
		    pos.XYZH2TrackPos(seg1.x, seg1.y, 0, seg1.road->GetGeometry(seg1.segmentIdx)->GetHdg());
			s1 = pos.GetS();

			roadmanager::Position pos_ = centralObject_->pos_;
			
			s2 = centralObject_->pos_.GetS();

			if (s1 <= s2) {
				ellipse_.lower            = seg1;
				ellipse_.upper.segmentIdx = -1;
			} else {
				ellipse_.upper            = seg1;
				ellipse_.lower.segmentIdx = -1;
			}
			break;
		}
		default:
		    return false;
	}
	return true;
}

void SwarmTrafficAction::despawn() {

	auto idPtr = vehiclesId_.begin();
	bool increase = true;
	while (idPtr < vehiclesId_.end()) {
		Object *vehicle = entities_->GetObjectById(*idPtr);
		roadmanager::Position pos;
        if (vehicle->pos_.GetH() == centralObject_->pos_.GetH()) {
            pos.XYZH2TrackPos(ellipse_.upper.x, ellipse_.upper.y, 0, centralObject_->pos_.GetH());
			if (vehicle->pos_.GetS() >= pos.GetS()) { 
			    entities_->removeObject(vehicle->name_);
				delete vehicle;
				idPtr = vehiclesId_.erase(idPtr);
				increase = false;
			}
		} else {
			pos.XYZH2TrackPos(ellipse_.lower.x, ellipse_.lower.y, 0, centralObject_->pos_.GetH());
			if (vehicle->pos_.GetS() <= pos.GetS()) {
			    entities_->removeObject(vehicle->name_);
				delete vehicle;
				idPtr = vehiclesId_.erase(idPtr);
				increase = false;
			}
		}

		if (increase) ++idPtr;
		increase = true;
	}
}