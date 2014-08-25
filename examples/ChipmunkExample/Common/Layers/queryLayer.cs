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
		cpVect start;
		cpVect end;

		//cpNearestPointQueryInfo nearestInfo;
		cpSegmentQueryInfo info;
		cpVect point;

		CCLabelTtf font;

		protected override void AddedToScene()
		{
			base.AddedToScene();


			start = new cpVect(320, 240);

			font = GetDefaultFontTtf("TEST");
			font.Position = new CCPoint(windowSize.Width * .5f, 10);
			InformationLayer.AddChild(font);
			font.Scale = .3f;

			space.iterations = 5;

			{ // add a fat segment
				var mass = 1;
				var length = 100;
				cpVect a = new cpVect(-length / 2, 0), b = new cpVect(length / 2, 0);

				var body = space.AddBody(new cpBody(mass, cp.momentForSegment(mass, a, b)));
				body.SetPosition(new cpVect(320, 340));

				space.AddShape(new cpSegmentShape(body, a, b, 20));
			}

			{ // add a static segment
				space.AddShape(new cpSegmentShape(space.StaticBody, new cpVect(320, 540), new cpVect(620, 240), 0));
			}

			{ // add a pentagon
				float mass = 1;
				int NUM_VERTS = 5;

				var verts = new float[NUM_VERTS * 2];
				for (var i = 0; i < NUM_VERTS * 2; i += 2)
				{
					float angle = -(float)Math.PI * i / (float)NUM_VERTS;
					verts[i] = 30 * cp.cpfcos(angle);
					verts[i + 1] = 30 * cp.cpfsin(angle);
				}

				var body = space.AddBody(new cpBody(mass, cp.momentForPoly(mass, verts, new cpVect(0, 0))));
				body.SetPosition(new cpVect(350 + 60, 220 + 60));

				space.AddShape(new cpPolyShape(body, verts,0.0f));
			}

			{ // add a circle
				var mass = 1;
				var r = 20;

				var body = space.AddBody(new cpBody(mass, cp.momentForCircle(mass, 0, r, new cpVect(0, 0))));
				body.SetPosition(new cpVect(320 + 100, 240 + 120));

				space.AddShape(new cpCircleShape(body, r, new cpVect(0, 0)));

			}

			Schedule();
		}


		public void QueryDraw()
		{

			if (!CCMouse.Instance.HasPosition)
				return;

			end = CCMouse.Instance.Position;

			string message = "Query: Dist(" + Math.Floor(cpVect.cpvdist(start, end)) + ") Point " + end.ToString() + ", ";

			info = this.space.SegmentQueryFirst(start, end, cp.ALL_LAYERS, cp.NO_GROUP);
			if (info != null)
			{
				//Console.WriteLine("Found >> info");

				point = info.HitPoint(start, end);
				message += "Segment Query: Dist(" + cp.cpffloor(info.HitDist(start, end)) + ") Normal " + info.normal.ToString();
			}
			else
			{
				message += "Segment Query: (None)";
			}

			font.Text = message;

			 this.space.NearestPointQuery(CCMouse.Instance.Position, 100, cp.ALL_LAYERS, cp.NO_GROUP);
			//if (nearestInfo != null)
			//	Console.WriteLine("Found >> nearestInfo");
		}

		public override void OnTouchesEnded(List<CCTouch> touches, CCEvent arg2)
		{
			base.OnTouchesEnded(touches, arg2);

		}

		public override void OnTouchesBegan(List<CCTouch> touches, CCEvent e)
		{
			base.OnTouchesBegan(touches, e);

			//nearestInfo = this.space.nearestPointQueryNearest(this.mouse, 100, cp.ALL_LAYERS, cp.NO_GROUP);
			//if (nearestInfo != null)
			//	Console.WriteLine("Found >> nearestInfo");

			//info = this.space.segmentQueryFirst(start, end, cp.ALL_LAYERS, cp.NO_GROUP);
			//if (info != null)
			//{
			//	Console.WriteLine("Found >> info");

			//	point = info.hitPoint(start, end);
			//	message += "Segment Query: Dist(" + cp.cpffloor(info.hitDist(start, end)) + ") Normal " + info.n.ToString();
			//}

			//QueryDraw();

		}


		public override void Update(float dt)
		{
			base.Update(dt);
			QueryDraw();
			space.Step(dt);
		}

		protected override void Draw()
		{
			base.Draw();

			m_debugDraw.Begin();

			//if (start != null && end != null)
			//	// Draw a green line from start to end.
			//	m_debugDraw.DrawSegment(start, end, cpColor.Green);


			//if (nearestInfo != null)
			//{
			//	lock (nearestInfo)
			//	{

			//		m_debugDraw.DrawSegment(CCMouse.Instance.Position, nearestInfo.p, cpColor.Green);
			//		// Draw a red bounding box around the shape under the mouse.
			//		if (nearestInfo.d < 0)
			//		{
			//			nearestInfo.shape.Draw(m_debugDraw);
			//			//m_debugDraw.DrawPolygon drawBB(nearestInfo.shape.getBB(), null, cpColor.Red);
			//			//m_debugDraw.DrawBB(nearestInfo.shape.getBB(), cpColor.Red);
			//		}
			//	}
			//}
			//if (info != null)
			//{
			//	lock (info)
			//	{
			//		// Draw red over the occluded part of the query
			//		m_debugDraw.DrawSegment(point, end, cpColor.Red);

			//		// Draw a little blue surface normal
			//		m_debugDraw.DrawSegment(point, cpVect.cpvadd(point, cpVect.cpvmult(info.n, 16)), cpColor.Blue);

			//	}


			//}

			m_debugDraw.End();
		}



	}
}
