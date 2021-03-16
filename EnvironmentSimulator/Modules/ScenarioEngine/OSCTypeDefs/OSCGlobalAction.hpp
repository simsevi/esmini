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

#pragma once
#include <iostream>
#include "OSCAction.hpp"
#include "CommonMini.hpp"
#include "Parameters.hpp"

namespace scenarioengine
{

	class OSCGlobalAction : public OSCAction
	{
	public:
		typedef enum
		{
			ENVIRONMENT,     // not supported yet
			ENTITY,          // not supported yet
			PARAMETER_SET,       
			INFRASTRUCTURE,  // not supported yet
			SWARM_TRAFFIC,       
		} Type;

		Type type_;

		OSCGlobalAction(OSCGlobalAction::Type type) : OSCAction(OSCAction::BaseType::GLOBAL), type_(type)
		{
			LOG("");
		}

		virtual void print()
		{
			LOG("Virtual, should be overridden");
		}

		virtual OSCGlobalAction* Copy()
		{
			LOG("Virtual, should be overridden");
			return 0;
		};

	};

	class ParameterSetAction : public OSCGlobalAction
	{
	public:
		std::string name_;
		std::string value_;
		Parameters* parameters_;

		ParameterSetAction() : OSCGlobalAction(OSCGlobalAction::Type::PARAMETER_SET), name_(""), value_(""), parameters_(0) {};

		ParameterSetAction(const ParameterSetAction& action) : OSCGlobalAction(OSCGlobalAction::Type::PARAMETER_SET)
		{
			name_ = action.name_;
			value_ = action.value_;
		}

		OSCGlobalAction* Copy()
		{
			ParameterSetAction* new_action = new ParameterSetAction(*this);
			return new_action;
		}

		void Start();

		void Step(double dt, double simTime);

		void print()
		{
			LOG("");
		}

	};

	class SwarmTrafficAction : public OSCGlobalAction
	{
	public:
		
		typedef struct {
			roadmanager::Road* road;
            int                segmentIdx;
			double             x;
			double             y;
		} segmentInfo;

		typedef struct {
			segmentInfo upper, lower;
		} curveInfo;
		
		SwarmTrafficAction() : OSCGlobalAction(OSCGlobalAction::Type::SWARM_TRAFFIC), centralObject_(0), i(0) {};

		SwarmTrafficAction(const SwarmTrafficAction& action) : OSCGlobalAction(OSCGlobalAction::Type::SWARM_TRAFFIC), i(0)
		{
		}

		OSCGlobalAction* Copy()
		{
			SwarmTrafficAction* new_action = new SwarmTrafficAction(*this);
			return new_action;
		}

		void Start();

		void Step(double dt, double simTime);

		void print()
		{
			LOG("");
		}

		void SetCentralObject(Object* centralObj) { centralObject_ = centralObj; }
		void SetInnerRadius(double innerRadius)   { innerRadius_   = innerRadius;}
		void SetSemiMajorAxes(double axes)        { semiMajorAxis_ = axes;       }
		void SetSemiMinorAxes(double axes)        { semiMinorAxis_ = axes;       }

        /*
		void GetCentralObject() { return centralObject_; }
		void GetInnerRadius()   { return innerRadius_  ; }
		void GetSemiMajorAxes() { return semiMajorAxis_; }
		void GetSemiMinorAxes() { return semiMinorAxis_; }
		*/

    private:

		Object* centralObject_;
		double innerRadius_, semiMajorAxis_, semiMinorAxis_;
		curveInfo circle_, ellipse_;
		roadmanager::OpenDrive* odrManager_;
		Entities entities_;
		size_t i;

		void initRoadSegments();
	};
}

