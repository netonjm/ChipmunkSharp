using ChipmunkSharp;
using CocosSharp;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace ChipmunkExample
{
	class stickyLayer : ChipmunkDemoLayer
	{



		ulong COLLISION_TYPE_STICKY = 1;

		float STICK_SENSOR_THICKNESS = 2.5f;


		public void PostStepAddJoint(object key, object data)
		{
			//	printf("Adding joint for %p\n", data);

			cpConstraint joint = (cpConstraint)key;
			space.AddConstraint(joint);
		}


		public bool StickyPreSolve(cpArbiter arb, object data)
		{
			// We want to fudge the collisions a bit to allow shapes to overlap more.
			// This simulates their squishy sticky surface, and more importantly
			// keeps them from separating and destroying the joint.

			// Track the deepest collision point and use that to determine if a rigid collision should occur.
			float deepest = cp.Infinity;

			// Grab the contact set and iterate over them.
			cpContactPointSet contacts = arb.GetContactPointSet();
			for (int i = 0; i < contacts.count; i++)
			{
				// Sink the contact points into the surface of each shape.
				contacts.points[i].pointA = cpVect.cpvsub(contacts.points[i].pointA, cpVect.cpvmult(contacts.normal, STICK_SENSOR_THICKNESS));
				contacts.points[i].pointB = cpVect.cpvadd(contacts.points[i].pointB, cpVect.cpvmult(contacts.normal, STICK_SENSOR_THICKNESS));
				deepest = cp.cpfmin(deepest, contacts.points[i].distance);// + 2.0f*STICK_SENSOR_THICKNESS);
			}

			// Set the new contact point data.
			arb.SetContactPointSet(ref contacts);

			// If the shapes are overlapping enough, then create a
			// joint that sticks them together at the first contact point.
			if (arb.GetUserData() == null && deepest <= 0.0f)
			{
				cpBody bodyA, bodyB;
				arb.GetBodies(out bodyA, out bodyB);

				// Create a joint at the contact point to hold the body in place.
				cpVect anchorA = bodyA.WorldToLocal(contacts.points[0].pointA);
				cpVect anchorB = bodyB.WorldToLocal(contacts.points[0].pointB);

				cpConstraint joint = new cpPivotJoint(bodyA, bodyB, anchorA, anchorB);

				// Give it a finite force for the stickyness.
				joint.SetMaxForce(3e3f);


				// Schedule a post-step() callback to add the joint.
				space.AddPostStepCallback(
					(s, o1, o2) => PostStepAddJoint(joint, null),
					joint, null);

				// Store the joint on the arbiter so we can remove it later.
				arb.SetUserData(joint);
			}

			// Position correction and velocity are handled separately so changing
			// the overlap distance alone won't prevent the collision from occuring.
			// Explicitly the collision for this frame if the shapes don't overlap using the new distance.
			return (deepest <= 0.0f);

			// Lots more that you could improve upon here as well:
			// * Modify the joint over time to make it plastic.
			// * Modify the joint in the post-step to make it conditionally plastic (like clay).
			// * Track a joint for the deepest contact point instead of the first.
			// * Track a joint for each contact point. (more complicated since you only get one data pointer).
		}



		public void StickySeparate(cpArbiter arb, object data)
		{
			cpConstraint joint = (cpConstraint)arb.GetUserData();

			if (joint != null)
			{
				// The joint won't be removed until the step is done.
				// Need to disable it so that it won't apply itself.
				// Setting the force to 0 will do just that

				joint.SetMaxForce(0.0f);

				space.AddPostStepCallback((s1, o1, o2) =>
					PostStepRemoveJoint(o1, o2), joint, null

					);
				// Perform the removal in a post-step() callback.

				// NULL out the reference to the joint.
				// Not required, but it's a good practice.
				arb.SetUserData(null);
			}
		}
		public void PostStepRemoveJoint(object key, object data)
		{
			//	printf("Removing joint for %p\n", data);
			cpConstraint joint = (cpConstraint)key;
			space.RemoveConstraint(joint);
		}


		public override void Update(float dt)
		{
			base.Update(dt);

			space.Step(dt);
		}

		public override void OnEnter()
		{
			base.OnEnter();


			SetSubTitle("Sticky collisions using the cpArbiter data pointer.");

			space.SetIterations(10);
			space.SetGravity(new cpVect(0, -1000));
			space.SetCollisionSlop(2.0f);

			cpBody staticBody = space.GetStaticBody();
			cpShape shape;

			// Create segments around the edge of the screen.
			shape = space.AddShape(new cpSegmentShape(staticBody, new cpVect(-340, -260), new cpVect(-340, 260), 20.0f));
			shape.SetElasticity(1.0f);
			shape.SetFriction(1.0f);
			shape.SetFilter(NOT_GRABBABLE_FILTER);

			shape = space.AddShape(new cpSegmentShape(staticBody, new cpVect(340, -260), new cpVect(340, 260), 20.0f));
			shape.SetElasticity(1.0f);
			shape.SetFriction(1.0f);
			shape.SetFilter(NOT_GRABBABLE_FILTER);

			shape = space.AddShape(new cpSegmentShape(staticBody, new cpVect(-340, -260), new cpVect(340, -260), 20.0f));
			shape.SetElasticity(1.0f);
			shape.SetFriction(1.0f);
			shape.SetFilter(NOT_GRABBABLE_FILTER);

			shape = space.AddShape(new cpSegmentShape(staticBody, new cpVect(-340, 260), new cpVect(340, 260), 20.0f));
			shape.SetElasticity(1.0f);
			shape.SetFriction(1.0f);
			shape.SetFilter(NOT_GRABBABLE_FILTER);

			for (int i = 0; i < 80; i++)
			{
				float mass = 0.15f;
				float radius = 10.0f;

				cpBody body = space.AddBody(new cpBody(mass, cp.MomentForCircle(mass, 0.0f, radius, cpVect.Zero)));

				body.SetPosition(
					new cpVect(cp.cpflerp(-150.0f, 150.0f, RandomHelper.frand()),
						cp.cpflerp(-150.0f, 150.0f, RandomHelper.frand()))
						);

				shape = space.AddShape(new cpCircleShape(body, radius + STICK_SENSOR_THICKNESS, cpVect.Zero));
				shape.SetFriction(0.9f);
				shape.SetCollisionType(COLLISION_TYPE_STICKY);

			}

			cpCollisionHandler handler = space.AddWildcardHandler(COLLISION_TYPE_STICKY);
			handler.preSolveFunc = (a, s, o) => StickyPreSolve(a, null);
			handler.separateFunc = (a, s, o) => StickySeparate(a, null);

			Schedule();
		}


	}
}
