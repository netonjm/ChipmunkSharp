using ChipmunkSharp;
using CocosSharp;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace ChipmunkExample
{
	class shatterLayer : ChipmunkDemoLayer
	{

		public struct WorleyContex
		{

			public int seed;
			public float cellSize;
			public int width, height;
			public cpBB bb;
			public cpVect focus;

			public WorleyContex(int seed, float cellSize, int width, int height, cpBB bb, cpVect focus)
			{
				// TODO: Complete member initialization
				this.seed = seed;
				this.cellSize = cellSize;
				this.width = width;
				this.height = height;
				this.bb = bb;
				this.focus = focus;
			}
		}

		const float DENSITY = (1f / 10000f);
		const int MAX_VERTEXES_PER_VORONOI = 16;


		static cpVect HashVect(int x, int y, int seed)
		{
			//	cpfloat border = 0.21f;
			float border = 0.05f;

			long h = (x * 1640531513 ^ y * 2654435789) + seed;

			return new cpVect(
				cp.cpflerp(border, 1.0f - border, (h & 0xFFFF) / 0xFFFF),
				 cp.cpflerp(border, 1.0f - border, ((h >> 16) & 0xFFFF) / 0xFFFF)
			);
		}


		cpVect WorleyPoint(int i, int j, ref WorleyContex context)
		{
			float size = context.cellSize;
			int width = context.width;
			int height = context.height;
			cpBB bb = context.bb;

			//	cpVect fv = cpv(0.5, 0.5);
			cpVect fv = HashVect(i, j, context.seed);

			return new cpVect(
				cp.cpflerp(bb.l, bb.r, 0.5f) + size * (i + fv.x - width * 0.5f),
				cp.cpflerp(bb.b, bb.t, 0.5f) + size * (j + fv.y - height * 0.5f)
			);
		}

		int ClipCell(cpShape shape, cpVect center, int i, int j, WorleyContex context, cpVect[] verts, cpVect[] clipped, int count)
		{
			cpVect other = WorleyPoint(i, j, ref context);
			//	printf("  other %dx%d: (% 5.2f, % 5.2f) ", i, j, other.x, other.y);

			cpPointQueryInfo queryInfo = null;
			if (shape.PointQuery(other, ref queryInfo) > 0.0f)
			{

				for (int x = 0; x < count; x++)
					clipped[x] = new cpVect(verts[x]);

				return count;
			}
			else
			{
				//		printf("clipped\n");
			}

			cpVect n = cpVect.cpvsub(other, center);
			float dist = cpVect.cpvdot(n, cpVect.cpvlerp(center, other, 0.5f));

			int clipped_count = 0;
			for (j = 0, i = count - 1; j < count; i = j, j++)
			{
				cpVect a = verts[i];
				float a_dist = cpVect.cpvdot(a, n) - dist;

				if (a_dist <= 0.0f)
				{
					clipped[clipped_count] = a;
					clipped_count++;
				}

				cpVect b = verts[j];
				float b_dist = cpVect.cpvdot(b, n) - dist;

				if (a_dist * b_dist < 0.0f)
				{
					float t = cp.cpfabs(a_dist) / (cp.cpfabs(a_dist) + cp.cpfabs(b_dist));

					clipped[clipped_count] = cpVect.cpvlerp(a, b, t);
					clipped_count++;
				}
			}

			return clipped_count;
		}



		public void ShatterShape(cpPolyShape shape, float cellSize, cpVect focus)
		{
			space.RemoveShape(shape);
			space.RemoveBody(shape.GetBody());

			cpBB bb = shape.bb;
			int width = (int)((bb.r - bb.l) / cellSize) + 1;
			int height = (int)((bb.t - bb.b) / cellSize) + 1;
			//	printf("Splitting as %dx%d\n", width, height);
			WorleyContex context = new WorleyContex((int)RandomHelper.frand(), cellSize, width, height, bb, focus);

			for (int i = 0; i < context.width; i++)
			{
				for (int j = 0; j < context.height; j++)
				{
					cpVect cell = WorleyPoint(i, j, ref context);

					cpPointQueryInfo cp = null;
					if (shape.PointQuery(cell, ref cp) < 0.0f)
					{
						ShatterCell(shape, cell, i, j, ref context);
					}
				}
			}

			//cpBodyFree(cpShapeGetBody(shape));
			//cpShapeFree(shape);
		}


		public override void OnEnter()
		{
			base.OnEnter();


			SetSubTitle("Right click something to shatter it.");

			space.SetIterations(30);
			space.SetGravity(new cpVect(0, -500));
			space.SetSleepTimeThreshold(0.5f);
			space.SetCollisionSlop(0.5f);

			cpBody body, staticBody = space.GetStaticBody();
			cpShape shape;

			// Create segments around the edge of the screen.
			shape = space.AddShape(new cpSegmentShape(staticBody, new cpVect(-1000, -240), new cpVect(1000, -240), 0.0f));

			shape.SetElasticity(1.0f);
			shape.SetFriction(1.0f);
			shape.SetFilter(NOT_GRABBABLE_FILTER);

			float width = 200.0f;
			float height = 200.0f;
			float mass = width * height * DENSITY;
			float moment = cp.MomentForBox(mass, width, height);

			body = space.AddBody(new cpBody(mass, moment));

			shape = space.AddShape(cpPolyShape.BoxShape(body, width, height, 0.0f));
			shape.SetFriction(0.6f);


			Schedule();

		}


		public override void Update(float dt)
		{
			base.Update(dt);

			space.Step(dt);

			if (CCMouse.Instance.rightclick || CCMouse.Instance.dblclick)
			{
				cpPointQueryInfo info = null;
				if (space.PointQueryNearest(CCMouse.Instance.Position, 0, GRAB_FILTER, ref info) != null)
				{
					cpBB bb = info.shape.GetBB();// cpShapeGetBB();
					float cell_size = cp.cpfmax(bb.r - bb.l, bb.t - bb.b) / 5.0f;
					if (cell_size > 5)
					{
						ShatterShape(info.shape as cpPolyShape, cell_size, CCMouse.Instance.Position);
					}
					else
					{
						//printf("Too small to splinter %f\n", cell_size);
					}
				}
			}

		}




		public void ShatterCell(cpPolyShape shape, cpVect cell, int cell_i, int cell_j, ref WorleyContex context)
		{
			//	printf("cell %dx%d: (% 5.2f, % 5.2f)\n", cell_i, cell_j, cell.x, cell.y);

			cpBody body = shape.body;// cpShapeGetBody(shape);

			cpVect[] ping = new cpVect[MAX_VERTEXES_PER_VORONOI]; // cpVect[ (cpVect*)alloca( * sizeof(cpVect));
			cpVect[] pong = new cpVect[MAX_VERTEXES_PER_VORONOI]; //(cpVect*)alloca(MAX_VERTEXES_PER_VORONOI * sizeof(cpVect));

			int count = shape.Count;// cpPolyShapeGetCount();
			count = (count > MAX_VERTEXES_PER_VORONOI ? MAX_VERTEXES_PER_VORONOI : count);

			for (int i = 0; i < count; i++)
			{
				ping[i] = body.LocalToWorld(shape.GetVert(i));
			}

			cpPointQueryInfo info = null;
			for (int i = 0; i < context.width; i++)
			{
				for (int j = 0; j < context.height; j++)
				{

					if (
						!(i == cell_i && j == cell_j) &&
						shape.PointQuery(cell, ref info) < 0
					)
					{

						count = ClipCell(shape, cell, i, j, context, ping, pong, count);

						for (int u = 0; u < pong.Length; u++)
							if (pong[u] != null)
								ping[u] = new cpVect(pong[u]);

					}
				}
			}

			cpVect centroid = cp.CentroidForPoly(count, ping);
			float mass = cp.AreaForPoly(count, ping, 0) * DENSITY;
			float moment = cp.MomentForPoly(mass, count, ping, cpVect.cpvneg(centroid), 0);

			cpBody new_body = space.AddBody(new cpBody(mass, moment));
			new_body.SetPosition(centroid);

			new_body.SetPosition(centroid);
			new_body.SetVelocity(body.GetVelocityAtLocalPoint(centroid));
			new_body.SetAngularVelocity(body.GetAngularVelocity());

			cpTransform transform = cpTransform.Translate(cpVect.cpvneg(centroid));
			cpShape new_shape = space.AddShape(new cpPolyShape(new_body, count, ping, transform, 0));
			// Copy whatever properties you have set on the original shape that are important
			new_shape.SetFriction(shape.GetFriction());
		}




	}
}
