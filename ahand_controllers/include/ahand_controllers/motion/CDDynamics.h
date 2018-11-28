/*
 * how to use

	CDDynamics *testDyn;
	testDyn = new CDDynamics(dim, dt, wn);

	testDyn->SetVelocityLimits(velLimits);
	testDyn->SetState(initial);
	testDyn->SetTarget(target);


	start loop
		// if new target is set
		testDyn->SetTarget(target);

		// update dynamics
		testDyn->Update();

		// get state
		testDyn->GetState(state);
	end loop
 */

#ifndef ORR_CDDYNAMICS_H
#define ORR_CDDYNAMICS_H

#include <Eigen/Dense>


typedef Eigen::VectorXd Vector;

// Critically Damped Dynamics

class CDDynamics {

    private :

        Vector mTarget;
        Vector mTargetVelocity;

        Vector mState;
        Vector mStateVelocity;

        Vector mVelocityLimits;
        Vector mPositionLimits;

        unsigned int mDim;
        double mWn;
        double mDT;

        double goal_t;
        double current_t;

        public :

            CDDynamics();

            CDDynamics(int dim, double dt, double Wn);

            void SetState(const Vector & Position);
            void SetState(const Vector & Position, const Vector & Velocity);
            void SetTarget(const Vector & target);
            void SetStateTarget(const Vector & Position, const Vector & Target);

            void SetDt(double dt);
            void SetWn(double Wn);
            void SetVelocityLimits(const Vector & velLimits);
            void RemoveVelocityLimits();
            void SetPositionLimits(const Vector & posLimits);
            void RemovePositionLimits();


            void GetTarget(Vector & target);

            void GetState(Vector & Position);
            void GetState(Vector & Position, Vector & Velocity);

            void Update();
            void Update(double dt);
            void Update(double dt, double muxVel);
        };

#endif //ORR_CDDYNAMICS_H
