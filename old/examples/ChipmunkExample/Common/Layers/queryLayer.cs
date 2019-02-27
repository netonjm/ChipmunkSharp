using CocosSharp;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using ChipmunkSharp;

namespace ChipmunkExample
{
	class queryLayer : ChipmunkDemoLayer
	{
		cpVect QUERY_START;
		cpVect start;
		cpVect end;
		CCLabelTtf font;

		cpSegmentQueryInfo segInfo;
		cpShape shapeInfo;
		cpPointQueryInfo nearestInfo;

		public override void OnEnter()
		{
			base.OnEnter();


			QUERY_START = new cpVect(320, 240);

			font = GetDefaultFontTtf("TEST");
			font.Position = new CCPoint((float)windowSize.Width * .5f, 10);
			InformationLayer.AddChild(font);
			font.Scale = .5f;

			space.SetIterations(5);

			{ // add a fat segment
				var mass = 1;
				var length = 100;
				cpVect a = new cpVect(-length / 2, 0), b = new cpVect(length / 2, 0);

				var body = space.AddBody(new cpBody(mass, cp.MomentForSegment(mass, a, b, 0.0f)));
				body.SetPosition(new cpVect(0.0f, 100.0f));

				space.AddShape(new cpSegmentShape(body, a, b, 20));
			}

			{ // add a static segment
				space.AddShape(new cpSegmentShape(space.GetStaticBody(), new cpVect(0, 300), new cpVect(300, 0), 0));
			}

			{ // add a pentagon
				float mass = 1;
				int NUM_VERTS = 5;

				cpVect[] verts = new cpVect[NUM_VERTS];
				for (int i = 0; i < NUM_VERTS; i++)
				{
					float angle = -2 * cp.M_PI * i / (NUM_VERTS);
					verts[i] = cpVect.cpv(30 * cp.cpfcos(angle), 30 * cp.cpfsin(angle));
				}


				var body = space.AddBody(new cpBody(mass, cp.MomentForPoly(mass, NUM_VERTS, verts, cpVect.Zero, 0.0f)));
				body.SetPosition(new cpVect(50.0f, 30.0f));

				space.AddShape(new cpPolyShape(body, NUM_VERTS, verts, cpTransform.Identity, 10f));
			}

			{ // add a circle
				var mass = 1;
				var r = 20;

				var body = space.AddBody(new cpBody(mass, cp.MomentForCircle(mass, 0, r, new cpVect(0, 0))));
				body.SetPosition(new cpVect(100.0f, 100.0f));

				space.AddShape(new cpCircleShape(body, r, cpVect.Zero));

			}

			Schedule();

		}

		public override void OnTouchesBegan(List<CCTouch> touches, CCEvent e)
		{
			base.OnTouchesBegan(touches, e);




		}

		public override void Update(float dt)
		{
			base.Update(dt);

			if (CCMouse.Instance.rightclick)
				QUERY_START = CCMouse.Instance.Position;

			start = QUERY_START;
			end = CCMouse.Instance.Position;
			float radius = 10;

			segInfo = null;

			m_debugDraw.DrawSegment(start, end, 1, cpColor.Green);


			shapeInfo = this.space.SegmentQueryFirst(start, end, radius, cpShape.FILTER_ALL, ref segInfo);
			if (shapeInfo != null)
			{
				font.Text = string.Format("Segment Query: Dist({0}) Normal({1},{2})", segInfo.alpha * cpVect.cpvdist(start, end), segInfo.normal.x, segInfo.normal.y);
			}
			else
			{
				font.Text = string.Format("Segment Query (None)");
			}

			nearestInfo = null;
			space.PointQueryNearest(CCMouse.Instance.Position, 100, cpShape.FILTER_ALL, ref nearestInfo);
			if (nearestInfo != null)
			{
				// Draw a grey line to the closest shape.
				m_debugDraw.Draw(CCMouse.Instance.Position, new cpColor(127, 127, 127, 255));
				m_debugDraw.DrawSegment(CCMouse.Instance.Position, nearestInfo.point, 1, new cpColor(127, 127, 127, 255));

				// Draw a red bounding box around the shape under the mouse.
				if (nearestInfo.distance < 0)
				{
					m_debugDraw.Draw(nearestInfo.shape.bb, cpColor.Red);
				}
			}


			space.Step(dt);
		}

		protected override void Draw()
		{
			base.Draw();

			if (shapeInfo != null)
			{
				cpVect point = segInfo.point;
				cpVect n = segInfo.normal;

				m_debugDraw.DrawSegment(cpVect.cpvlerp(start, end, segInfo.alpha),
					end, 1, cpColor.Blue);
			}



		}


	}
}
