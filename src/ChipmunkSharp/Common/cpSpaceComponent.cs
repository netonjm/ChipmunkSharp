/* Copyright (c) 2007 Scott Lembcke ported by Jose Medrano (@netonjm)
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:
  
  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.
  
  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
 */
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace ChipmunkSharp
{
	public partial class cpSpace
	{

		public void ActivateBody(cpBody body)
		{

			cp.AssertHard(body.bodyType == cpBodyType.DYNAMIC, "Internal error: Attempting to deactivate a non-dynamic body.");

			if (this.IsLocked)
			{
				// cpSpaceActivateBody() is called again once the space is unlocked
				if (!this.rousedBodies.Contains(body))
					this.rousedBodies.Add(body);
			}
			else
			{

				cp.AssertSoft(body.nodeRoot == null &&
					body.nodeNext == null, "Internal error: Activating body non-NULL node pointers.");


				this.dynamicBodies.Add(body);

				body.eachShape((s, o) =>
				{
					this.staticShapes.Remove(s.hashid);
					this.dynamicShapes.Insert(s.hashid, s);
				}, null);


				body.eachArbiter((arb, o) =>
				{
					cpBody bodyA = arb.body_a;

					// Arbiters are shared between two bodies that are always woken up together.
					// You only want to restore the arbiter once, so bodyA is arbitrarily chosen to own the arbiter.
					// The edge case is when static bodies are involved as the static bodies never actually sleep.
					// If the static body is bodyB then all is good. If the static body is bodyA, that can easily be checked.
					if (body == bodyA || bodyA.bodyType == cpBodyType.STATIC)
					{
						cpShape a = arb.a, b = arb.b;
						this.cachedArbiters.Add(cp.CP_HASH_PAIR(a.hashid, b.hashid), arb);

						// Update the arbiter's state
						arb.stamp = this.stamp;
						arb.handler = this.LookupHandler(a.type, b.type, defaultHandler);
						this.arbiters.Add(arb);
					}

				}, null);

				body.eachConstraint((constraint, o) =>
				{
					var bodyA = constraint.a;
					if (body == bodyA || bodyA.bodyType == cpBodyType.STATIC)
						this.constraints.Add(constraint);

				}, null);

			}


		}

		public void DeactivateBody(cpBody body)
		{

			cp.AssertHard(body.bodyType == cpBodyType.DYNAMIC, "Internal error: Attempting to deactivate a non-dynamic body.");

			this.dynamicBodies.Remove(body);

			body.eachShape((shape, o) =>
				{
					this.dynamicShapes.Remove(shape.hashid);
					this.staticShapes.Insert(shape.hashid, shape);
				}, null);


			body.eachArbiter((arb, o) =>
			{

				var bodyA = arb.body_a;

				if (body == bodyA || bodyA.bodyType == cpBodyType.STATIC)
				{
					this.UncacheArbiter(arb);
				}

			}, null);


			body.eachConstraint((constraint, o) =>
			{
				var bodyA = constraint.a;
				if (body == bodyA || bodyA.bodyType == cpBodyType.STATIC)
					this.constraints.Remove(constraint);

			}, null);

		}

		public void ProcessComponents(float dt)
		{

			var sleep = (this.sleepTimeThreshold != cp.Infinity);
			var bodies = this.dynamicBodies;

			// These checks can be removed at some stage (if DEBUG == undefined)
			for (var i = 0; i < bodies.Count; i++)
			{
				var body = bodies[i];

				cp.AssertSoft(body.nodeNext == null, "Internal Error: Dangling next pointer detected in contact graph.");
				cp.AssertSoft(body.nodeRoot == null, "Internal Error: Dangling root pointer detected in contact graph.");
			}

			// Calculate the kinetic energy of all the bodies
			if (sleep)
			{
				var dv = this.idleSpeedThreshold;
				var dvsq = (dv != 0 ? dv * dv : cpVect.cpvlengthsq(this.gravity) * dt * dt);

				for (var i = 0; i < bodies.Count; i++)
				{
					// TODO should make a separate array for kinematic bodies.
					if (bodies[i].bodyType != cpBodyType.DYNAMIC)
						continue;

					// Need to deal with infinite mass objects
					var keThreshold = (dvsq > 0 ? bodies[i].m * dvsq : 0.0f);
					bodies[i].nodeIdleTime = (bodies[i].KineticEnergy() > keThreshold ? 0 : bodies[i].nodeIdleTime + dt);
				}
			}

			// Awaken any sleeping bodies found and then push arbiters to the bodies' lists.

			List<cpArbiter> arbiters = this.arbiters; // new List<cpArbiter>();
			var count = arbiters.Count; //FIX: we cannot read the count values of the array because it changes inside

			for (int i = 0; i < count; i++)
			{
				cpArbiter arb = arbiters[i];
				cpBody a = arb.body_a, b = arb.body_b;

				if (sleep)
				{

					if (b.bodyType == cpBodyType.KINEMATIC || a.IsSleeping())
						a.Activate();

					if (a.bodyType == cpBodyType.KINEMATIC || b.IsSleeping())
						b.Activate();
				}

				a.PushArbiter(arb);
				b.PushArbiter(arb);
			}

			if (sleep)
			{
				// Bodies should be held active if connected by a joint to a non-static rouge body.
				var constraints = this.constraints;
				for (var i = 0; i < constraints.Count; i++)
				{
					cpConstraint constraint = constraints[i];
					cpBody a = constraint.a, b = constraint.b;

					if (b.bodyType == cpBodyType.KINEMATIC)
						a.Activate();

					if (a.bodyType == cpBodyType.KINEMATIC)
						b.Activate();
				}

				// Generate components and deactivate sleeping ones
				for (var i = 0; i < bodies.Count; )
				{
					var body = bodies[i];

					if (cp.ComponentRoot(body) == null)
					{
						// Body not in a component yet. Perform a DFS to flood fill mark 
						// the component in the contact graph using this body as the root.
						FloodFillComponent(body, body);

						// Check if the component should be put to sleep.
						if (!ComponentActive(body, this.sleepTimeThreshold))
						{
							this.sleepingComponents.Add(body);
							//CP_BODY_FOREACH_COMPONENT
							for (var other = body; other != null; other = other.nodeNext)
							{
								this.DeactivateBody(other);
							}

							// deactivateBody() removed the current body from the list.
							// Skip incrementing the index counter.
							continue;
						}
					}

					i++;

					// Only sleeping bodies retain their component node pointers.
					body.nodeRoot = null;
					body.nodeNext = null;
				}
			}
		}

		public static bool ComponentActive(cpBody root, float threshold)
		{
			//CP_BODY_FOREACH_COMPONENT
			for (var body = root; body != null; body = body.nodeNext)
			{
				if (body.nodeIdleTime < threshold)
					return true;
			}

			return false;
		}


		public static void FloodFillComponent(cpBody root, cpBody body)
		{

			// Kinematic bodies cannot be put to sleep and prevent bodies they are touching from sleeping.
			// Static bodies are effectively sleeping all the time.
			if (body.bodyType == cpBodyType.DYNAMIC)
			{
				cpBody other_root = cp.ComponentRoot(body);
				if (other_root == null)
				{
					cp.componentAdd(root, body);
					body.eachArbiter((arb, o) =>
					{
						FloodFillComponent(root, (body == arb.body_a ?
							arb.body_b : arb.body_a));

					}, null);

					body.eachConstraint((constraint, o) =>
					{

						FloodFillComponent(root,
							(body == constraint.a ? constraint.b : constraint.a));

					}, null);

				}
				else
				{
					cp.AssertSoft(other_root == root, "Internal Error: Inconsistency dectected in the contact graph.");
				}
			}

		}


	}
}
