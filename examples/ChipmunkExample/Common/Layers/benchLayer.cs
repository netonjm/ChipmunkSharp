using ChipmunkSharp;
using CocosSharp;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace ChipmunkExample
{
    class benchLayer : ChipmunkDemoLayer
    {
        public static float bevel = 1.0f;

        static Func<cpSpace, cpSpace>[] bench_list = new Func<cpSpace, cpSpace>[] {
			SimpleTerrainCircles_100,
			SimpleTerrainCircles_500,
			SimpleTerrainCircles_1000,
		    SimpleTerrainBoxes_100,
			SimpleTerrainBoxes_500,
			SimpleTerrainBoxes_1000,
			SimpleTerrainVCircles_200,
			SimpleTerrainHexagons_100,
			SimpleTerrainHexagons_500,
			SimpleTerrainHexagons_1000,
			SimpleTerrainVBoxes_200,
			SimpleTerrainVHexagons_200,
			ComplexTerrainCircles_1000,
			ComplexTerrainHexagons_1000,
			BouncyTerrainCircles_500,
			BouncyTerrainHexagons_500,
			NoCollide,
		};

        public override void OnEnter()
        {
            base.OnEnter();
            CreateSubScenePlayer(bench_list.Length); //Create a subscene player with the array length
            Schedule();
            RefreshSubScene();
        }

        public override void ChangeActualSubScene(int index)
        {
            bench_list[index](space);
        }

        static cpVect[] simple_terrain_verts = new cpVect[] {
	new cpVect(350.00f, 425.07f), new cpVect(336.00f, 436.55f),new cpVect(272.00f, 435.39f), new cpVect(258.00f, 427.63f), new cpVect(225.28f, 420.00f), new cpVect(202.82f, 396.00f),
	new cpVect(191.81f, 388.00f), new cpVect(189.00f, 381.89f),new cpVect(173.00f, 380.39f), new cpVect(162.59f, 368.00f), new cpVect(150.47f, 319.00f), new cpVect(128.00f, 311.55f),
	new cpVect(119.14f, 286.00f), new cpVect(126.84f, 263.00f),new cpVect(120.56f, 227.00f), new cpVect(141.14f, 178.00f), new cpVect(137.52f, 162.00f), new cpVect(146.51f, 142.00f),
	new cpVect(156.23f, 136.00f), new cpVect(158.00f, 118.27f),new cpVect(170.00f, 100.77f), new cpVect(208.43f,  84.00f), new cpVect(224.00f,  69.65f), new cpVect(249.30f,  68.00f),
	new cpVect(257.00f,  54.77f), new cpVect(363.00f,  45.94f),new cpVect(374.15f,  54.00f), new cpVect(386.00f,  69.60f), new cpVect(413.00f,  70.73f), new cpVect(456.00f,  84.89f),
	new cpVect(468.09f,  99.00f), new cpVect(467.09f, 123.00f),new cpVect(464.92f, 135.00f), new cpVect(469.00f, 141.03f), new cpVect(497.00f, 148.67f), new cpVect(513.85f, 180.00f),
	new cpVect(509.56f, 223.00f), new cpVect(523.51f, 247.00f),new cpVect(523.00f, 277.00f), new cpVect(497.79f, 311.00f), new cpVect(478.67f, 348.00f), new cpVect(467.90f, 360.00f),
	new cpVect(456.76f, 382.00f), new cpVect(432.95f, 389.00f),new cpVect(417.00f, 411.32f), new cpVect(373.00f, 433.19f), new cpVect(361.00f, 430.02f), new cpVect(350.00f, 425.07f),
    };

        static public void add_circle(cpSpace space, int index, float radius)
        {
            float mass = radius * radius / 25.0f;
            cpBody body = space.AddBody(new cpBody(mass, cp.MomentForCircle(mass, 0.0f, radius, cpVect.Zero)));
            //	cpBody body = cpSpaceAddBody(space, cpBodyInit(&bodies[i], mass, cpMomentForCircle(mass, 0.0f, radius, cpVect.Zero)));
            body.SetPosition(cpVect.cpvmult(cp.frand_unit_circle(), 180.0f));

            cpShape shape = space.AddShape(new cpCircleShape(body, radius, cpVect.Zero));
            //	cpShape shape = cpSpaceAddShape(space, cpCircleShapeInit(&circles[i], body, radius, cpVect.Zero));
            shape.SetElasticity(0.0f); shape.SetFriction(0.9f);
        }

        static public void add_box(cpSpace space, int index, float size)
        {
            float mass = size * size / 100.0f;

            cpBody body = space.AddBody(new cpBody(mass, cp.MomentForBox(mass, size, size)));
            //	cpBody body = cpSpaceAddBody(space, cpBodyInit(&bodies[i], mass, cpMomentForBox(mass, size, size)));
            body.SetPosition(cpVect.cpvmult(cp.frand_unit_circle(), 180.0f));
            cpPolyShape shape = space.AddShape(cpPolyShape.BoxShape(body, size - bevel * 2f, size - bevel * 2f, 0f)) as cpPolyShape;
            shape.SetRadius(bevel);
            shape.SetElasticity(0.0f);
            shape.SetFriction(0.9f);
        }

        static public void add_hexagon(cpSpace space, int index, float radius)
        {

            cpVect[] hexagon = new cpVect[6];
            for (int i = 0; i < 6; i++)
            {
                float angle = -(float)Math.PI * 2.0f * i / 6.0f;
                hexagon[i] = cpVect.cpvmult(cpVect.cpv(cp.cpfcos(angle), cp.cpfsin(angle)), radius - bevel);
            }

            float mass = radius * radius;

            cpBody body = space.AddBody(new cpBody(mass, cp.MomentForPoly(mass, 6, hexagon, cpVect.Zero, 0.0f)));
            body.SetPosition(cpVect.cpvmult(cp.frand_unit_circle(), 180.0f));

            cpPolyShape shape = space.AddShape(new cpPolyShape(body, 6, hexagon, cpTransform.Identity, bevel)) as cpPolyShape;
            shape.SetElasticity(0.0f);
            shape.SetFriction(0.9f);
        }

        static public cpSpace SetupSpace_simpleTerrain(cpSpace space)
        {
            SetSubTitle("SetupSpace_simpleTerrain");
            space.SetIterations(10);
            space.SetGravity(new cpVect(0, -100));
            space.SetCollisionSlop(0.5f);

            cpVect offset = new cpVect(-320, -240);
            for (int i = 0; i < (simple_terrain_verts.Length - 1); i++)
            {
                cpVect a = simple_terrain_verts[i], b = simple_terrain_verts[i + 1];
                space.AddShape(new cpSegmentShape(space.GetStaticBody(), cpVect.cpvadd(a, offset), cpVect.cpvadd(b, offset), 0.0f));
            }
            return space;
        }

        // SimpleTerrain constant sized objects
        static public cpSpace SimpleTerrainCircles_1000(cpSpace space)
        {
            SetSubTitle("SimpleTerrainCircles_1000");
            space = SetupSpace_simpleTerrain(space);
            for (int i = 0; i < 1000; i++)
                add_circle(space, i, 5.0f);
            return space;
        }

        static public cpSpace SimpleTerrainCircles_500(cpSpace space)
        {
            SetSubTitle("SimpleTerrainCircles_500");
            space = SetupSpace_simpleTerrain(space);
            for (int i = 0; i < 500; i++)
                add_circle(space, i, 5.0f);
            return space;
        }

        static public cpSpace SimpleTerrainCircles_100(cpSpace space)
        {
            SetSubTitle("SimpleTerrainCircles_100");
            space = SetupSpace_simpleTerrain(space);
            for (int i = 0; i < 100; i++)
                add_circle(space, i, 5.0f);
            return space;
        }

        static public cpSpace SimpleTerrainBoxes_1000(cpSpace space)
        {
            SetSubTitle("SimpleTerrainBoxes_1000");
            space = SetupSpace_simpleTerrain(space);
            for (int i = 0; i < 1000; i++)
                add_box(space, i, 10.0f);
            return space;
        }

        static public cpSpace SimpleTerrainBoxes_500(cpSpace space)
        {
            SetSubTitle("SimpleTerrainBoxes_500");
            space = SetupSpace_simpleTerrain(space);
            for (int i = 0; i < 500; i++)
                add_box(space, i, 10.0f);
            return space;
        }

        static public cpSpace SimpleTerrainBoxes_100(cpSpace space)
        {
            SetSubTitle("SimpleTerrainBoxes_100");
            space = SetupSpace_simpleTerrain(space);
            for (int i = 0; i < 100; i++)
                add_box(space, i, 10.0f);
            return space;
        }

        static public cpSpace SimpleTerrainHexagons_1000(cpSpace space)
        {
            SetSubTitle("SimpleTerrainHexagons_1000");
            space = SetupSpace_simpleTerrain(space);
            for (int i = 0; i < 1000; i++)
                add_hexagon(space, i, 5.0f);
            return space;
        }

        static public cpSpace SimpleTerrainHexagons_500(cpSpace space)
        {
            SetSubTitle("SimpleTerrainHexagons_500");
            space = SetupSpace_simpleTerrain(space);
            for (int i = 0; i < 500; i++)
                add_hexagon(space, i, 5.0f);
            return space;
        }

        static public cpSpace SimpleTerrainHexagons_100(cpSpace space)
        {
            SetSubTitle("SimpleTerrainHexagons_100");
            space = SetupSpace_simpleTerrain(space);
            for (int i = 0; i < 100; i++)
                add_hexagon(space, i, 5.0f);
            return space;
        }

        // SimpleTerrain variable sized objects
        static public float rand_size()
        {

            return cp.cpfpow(1.5f, cp.cpflerp(-1.5f, 3.5f, cp.frand()));
        }

        static public cpSpace SimpleTerrainVCircles_200(cpSpace space)
        {
            SetSubTitle("SimpleTerrainVCircles_200");
            space = SetupSpace_simpleTerrain(space);
            for (int i = 0; i < 200; i++)
                add_circle(space, i, 5.0f * rand_size());
            return space;
        }

        static public cpSpace SimpleTerrainVBoxes_200(cpSpace space)
        {
            SetSubTitle("SimpleTerrainVBoxes_200");
            space = SetupSpace_simpleTerrain(space);
            for (int i = 0; i < 200; i++)
                add_box(space, i, 8.0f * rand_size());
            return space;
        }

        static public cpSpace SimpleTerrainVHexagons_200(cpSpace space)
        {
            SetSubTitle("SimpleTerrainVHexagons_200");
            space = SetupSpace_simpleTerrain(space);
            for (int i = 0; i < 200; i++)
                add_hexagon(space, i, 5.0f * rand_size());
            return space;
        }

        // ComplexTerrain
        static cpVect[] complex_terrain_verts = new cpVect[] {
	new cpVect( 46.78f, 479.00f), new cpVect( 35.00f, 475.63f), new cpVect( 27.52f, 469.00f), new cpVect( 23.52f, 455.00f), new cpVect( 23.78f, 441.00f), new cpVect( 28.41f, 428.00f), new cpVect( 49.61f, 394.00f), new cpVect( 59.00f, 381.56f), new cpVect( 80.00f, 366.03f), new cpVect( 81.46f, 358.00f), new cpVect( 86.31f, 350.00f), new cpVect( 77.74f, 320.00f),
	new cpVect( 70.26f, 278.00f), new cpVect( 67.51f, 270.00f), new cpVect( 58.86f, 260.00f), new cpVect( 57.19f, 247.00f), new cpVect( 38.00f, 235.60f), new cpVect( 25.76f, 221.00f), new cpVect( 24.58f, 209.00f), new cpVect( 27.63f, 202.00f), new cpVect( 31.28f, 198.00f), new cpVect( 40.00f, 193.72f), new cpVect( 48.00f, 193.73f), new cpVect( 55.00f, 196.70f),
	new cpVect( 62.10f, 204.00f), new cpVect( 71.00f, 209.04f), new cpVect( 79.00f, 206.55f), new cpVect( 88.00f, 206.81f), new cpVect( 95.88f, 211.00f), new cpVect(103.00f, 220.49f), new cpVect(131.00f, 220.51f), new cpVect(137.00f, 222.66f), new cpVect(143.08f, 228.00f), new cpVect(146.22f, 234.00f), new cpVect(147.08f, 241.00f), new cpVect(145.45f, 248.00f),
	new cpVect(142.31f, 253.00f), new cpVect(132.00f, 259.30f), new cpVect(115.00f, 259.70f), new cpVect(109.28f, 270.00f), new cpVect(112.91f, 296.00f), new cpVect(119.69f, 324.00f), new cpVect(129.00f, 336.26f), new cpVect(141.00f, 337.59f), new cpVect(153.00f, 331.57f), new cpVect(175.00f, 325.74f), new cpVect(188.00f, 325.19f), new cpVect(235.00f, 317.46f),
	new cpVect(250.00f, 317.19f), new cpVect(255.00f, 309.12f), new cpVect(262.62f, 302.00f), new cpVect(262.21f, 295.00f), new cpVect(248.00f, 273.59f), new cpVect(229.00f, 257.93f), new cpVect(221.00f, 255.48f), new cpVect(215.00f, 251.59f), new cpVect(210.79f, 246.00f), new cpVect(207.47f, 234.00f), new cpVect(203.25f, 227.00f), new cpVect(179.00f, 205.90f),
	new cpVect(148.00f, 189.54f), new cpVect(136.00f, 181.45f), new cpVect(120.00f, 180.31f), new cpVect(110.00f, 181.65f), new cpVect( 95.00f, 179.31f), new cpVect( 63.00f, 166.96f), new cpVect( 50.00f, 164.23f), new cpVect( 31.00f, 154.49f), new cpVect( 19.76f, 145.00f), new cpVect( 15.96f, 136.00f), new cpVect( 16.65f, 127.00f), new cpVect( 20.57f, 120.00f),
	new cpVect( 28.00f, 114.63f), new cpVect( 40.00f, 113.67f), new cpVect( 65.00f, 127.22f), new cpVect( 73.00f, 128.69f), new cpVect( 81.96f, 120.00f), new cpVect( 77.58f, 103.00f), new cpVect( 78.18f,  92.00f), new cpVect( 59.11f,  77.00f), new cpVect( 52.00f,  67.29f), new cpVect( 31.29f,  55.00f), new cpVect( 25.67f,  47.00f), new cpVect( 24.65f,  37.00f),
	new cpVect( 27.82f,  29.00f), new cpVect( 35.00f,  22.55f), new cpVect( 44.00f,  20.35f), new cpVect( 49.00f,  20.81f), new cpVect( 61.00f,  25.69f), new cpVect( 79.00f,  37.81f), new cpVect( 88.00f,  49.64f), new cpVect( 97.00f,  56.65f), new cpVect(109.00f,  49.61f), new cpVect(143.00f,  38.96f), new cpVect(197.00f,  37.27f), new cpVect(215.00f,  35.30f),
	new cpVect(222.00f,  36.65f), new cpVect(228.42f,  41.00f), new cpVect(233.30f,  49.00f), new cpVect(234.14f,  57.00f), new cpVect(231.00f,  65.80f), new cpVect(224.00f,  72.38f), new cpVect(218.00f,  74.50f), new cpVect(197.00f,  76.62f), new cpVect(145.00f,  78.81f), new cpVect(123.00f,  87.41f), new cpVect(117.59f,  98.00f), new cpVect(117.79f, 104.00f),
	new cpVect(119.00f, 106.23f), new cpVect(138.73f, 120.00f), new cpVect(148.00f, 129.50f), new cpVect(158.50f, 149.00f), new cpVect(203.93f, 175.00f), new cpVect(229.00f, 196.60f), new cpVect(238.16f, 208.00f), new cpVect(245.20f, 221.00f), new cpVect(275.45f, 245.00f), new cpVect(289.00f, 263.24f), new cpVect(303.60f, 287.00f), new cpVect(312.00f, 291.57f),
	new cpVect(339.25f, 266.00f), new cpVect(366.33f, 226.00f), new cpVect(363.43f, 216.00f), new cpVect(364.13f, 206.00f), new cpVect(353.00f, 196.72f), new cpVect(324.00f, 181.05f), new cpVect(307.00f, 169.63f), new cpVect(274.93f, 156.00f), new cpVect(256.00f, 152.48f), new cpVect(228.00f, 145.13f), new cpVect(221.09f, 142.00f), new cpVect(214.87f, 135.00f),
	new cpVect(212.67f, 127.00f), new cpVect(213.81f, 119.00f), new cpVect(219.32f, 111.00f), new cpVect(228.00f, 106.52f), new cpVect(236.00f, 106.39f), new cpVect(290.00f, 119.40f), new cpVect(299.33f, 114.00f), new cpVect(300.52f, 109.00f), new cpVect(300.30f,  53.00f), new cpVect(301.46f,  47.00f), new cpVect(305.00f,  41.12f), new cpVect(311.00f,  36.37f),
	new cpVect(317.00f,  34.43f), new cpVect(325.00f,  34.81f), new cpVect(334.90f,  41.00f), new cpVect(339.45f,  50.00f), new cpVect(339.82f, 132.00f), new cpVect(346.09f, 139.00f), new cpVect(350.00f, 150.26f), new cpVect(380.00f, 167.38f), new cpVect(393.00f, 166.48f), new cpVect(407.00f, 155.54f), new cpVect(430.00f, 147.30f), new cpVect(437.78f, 135.00f),
	new cpVect(433.13f, 122.00f), new cpVect(410.23f,  78.00f), new cpVect(401.59f,  69.00f), new cpVect(393.48f,  56.00f), new cpVect(392.80f,  44.00f), new cpVect(395.50f,  38.00f), new cpVect(401.00f,  32.49f), new cpVect(409.00f,  29.41f), new cpVect(420.00f,  30.84f), new cpVect(426.92f,  36.00f), new cpVect(432.32f,  44.00f), new cpVect(439.49f,  51.00f),
	new cpVect(470.13f, 108.00f), new cpVect(475.71f, 124.00f), new cpVect(483.00f, 130.11f), new cpVect(488.00f, 139.43f), new cpVect(529.00f, 139.40f), new cpVect(536.00f, 132.52f), new cpVect(543.73f, 129.00f), new cpVect(540.47f, 115.00f), new cpVect(541.11f, 100.00f), new cpVect(552.18f,  68.00f), new cpVect(553.78f,  47.00f), new cpVect(559.00f,  39.76f),
	new cpVect(567.00f,  35.52f), new cpVect(577.00f,  35.45f), new cpVect(585.00f,  39.58f), new cpVect(591.38f,  50.00f), new cpVect(591.67f,  66.00f), new cpVect(590.31f,  79.00f), new cpVect(579.76f, 109.00f), new cpVect(582.25f, 119.00f), new cpVect(583.66f, 136.00f), new cpVect(586.45f, 143.00f), new cpVect(586.44f, 151.00f), new cpVect(580.42f, 168.00f),
	new cpVect(577.15f, 173.00f), new cpVect(572.00f, 177.13f), new cpVect(564.00f, 179.49f), new cpVect(478.00f, 178.81f), new cpVect(443.00f, 184.76f), new cpVect(427.10f, 190.00f), new cpVect(424.00f, 192.11f), new cpVect(415.94f, 209.00f), new cpVect(408.82f, 228.00f), new cpVect(405.82f, 241.00f), new cpVect(411.00f, 250.82f), new cpVect(415.00f, 251.50f),
	new cpVect(428.00f, 248.89f), new cpVect(469.00f, 246.29f), new cpVect(505.00f, 246.49f), new cpVect(533.00f, 243.60f), new cpVect(541.87f, 248.00f), new cpVect(547.55f, 256.00f), new cpVect(548.48f, 267.00f), new cpVect(544.00f, 276.00f), new cpVect(534.00f, 282.24f), new cpVect(513.00f, 285.46f), new cpVect(468.00f, 285.76f), new cpVect(402.00f, 291.70f),
	new cpVect(392.00f, 290.29f), new cpVect(377.00f, 294.46f), new cpVect(367.00f, 294.43f), new cpVect(356.44f, 304.00f), new cpVect(354.22f, 311.00f), new cpVect(362.00f, 321.36f), new cpVect(390.00f, 322.44f), new cpVect(433.00f, 330.16f), new cpVect(467.00f, 332.76f), new cpVect(508.00f, 347.64f), new cpVect(522.00f, 357.67f), new cpVect(528.00f, 354.46f),
	new cpVect(536.00f, 352.96f), new cpVect(546.06f, 336.00f), new cpVect(553.47f, 306.00f), new cpVect(564.19f, 282.00f), new cpVect(567.84f, 268.00f), new cpVect(578.72f, 246.00f), new cpVect(585.00f, 240.97f), new cpVect(592.00f, 238.91f), new cpVect(600.00f, 239.72f), new cpVect(606.00f, 242.82f), new cpVect(612.36f, 251.00f), new cpVect(613.35f, 263.00f),
	new cpVect(588.75f, 324.00f), new cpVect(583.25f, 350.00f), new cpVect(572.12f, 370.00f), new cpVect(575.45f, 378.00f), new cpVect(575.20f, 388.00f), new cpVect(589.00f, 393.81f), new cpVect(599.20f, 404.00f), new cpVect(607.14f, 416.00f), new cpVect(609.96f, 430.00f), new cpVect(615.45f, 441.00f), new cpVect(613.44f, 462.00f), new cpVect(610.48f, 469.00f),
	new cpVect(603.00f, 475.63f), new cpVect(590.96f, 479.00f), 
};
        static public cpSpace ComplexTerrainCircles_1000(cpSpace space)
        {
            space.SetIterations(10);
            space.SetGravity(new cpVect(0, -100));
            space.SetCollisionSlop(0.5f);

            cpVect offset = new cpVect(-320, -240);
            for (int i = 0; i < (complex_terrain_verts.Length - 1); i++)
            {
                cpVect a = complex_terrain_verts[i], b = complex_terrain_verts[i + 1];
                space.AddShape(new cpSegmentShape(space.GetStaticBody(), cpVect.cpvadd(a, offset), cpVect.cpvadd(b, offset), 0.0f));
            }

            for (int i = 0; i < 1000; i++)
            {
                float radius = 5.0f;
                float mass = radius * radius;
                cpBody body = space.AddBody(new cpBody(mass, cp.MomentForCircle(mass, 0.0f, radius, cpVect.Zero)));
                body.SetPosition(cpVect.cpvadd(cpVect.cpvmult(cp.frand_unit_circle(), 180.0f), new cpVect(0.0f, 300.0f)));
                cpShape shape = space.AddShape(new cpCircleShape(body, radius, cpVect.Zero));
                shape.SetElasticity(0.0f); shape.SetFriction(0.0f);
            }

            return space;
        }

        static public cpSpace ComplexTerrainHexagons_1000(cpSpace space)
        {
            SetSubTitle("ComplexTerrainHexagons_1000");
            space.SetIterations(10);
            space.SetGravity(new cpVect(0, -100));
            space.SetCollisionSlop(0.5f);

            cpVect offset = new cpVect(-320, -240);
            for (int i = 0; i < (complex_terrain_verts.Length - 1); i++)
            {
                cpVect a = complex_terrain_verts[i], b = complex_terrain_verts[i + 1];
                space.AddShape(new cpSegmentShape(space.GetStaticBody(), cpVect.cpvadd(a, offset), cpVect.cpvadd(b, offset), 0.0f));
            }

            float radius = 5.0f;

            cpVect[] hexagon = new cpVect[6];
            for (int i = 0; i < 6; i++)
            {
                float angle = -(float)Math.PI * 2.0f * i / 6.0f;
                hexagon[i] = cpVect.cpvmult(cpVect.cpv(cp.cpfcos(angle), cp.cpfsin(angle)), radius - bevel);
            }

            for (int i = 0; i < 1000; i++)
            {
                float mass = radius * radius;
                cpBody body = space.AddBody(new cpBody(mass, cp.MomentForPoly(mass, 6, hexagon, cpVect.Zero, 0.0f)));
                body.SetPosition(cpVect.cpvadd(cpVect.cpvmult(cp.frand_unit_circle(), 180.0f), new cpVect(0.0f, 300.0f)));

                cpShape shape = space.AddShape(new cpPolyShape(body, 6, hexagon, cpTransform.Identity, bevel));
                shape.SetElasticity(0.0f); shape.SetFriction(0.0f);
            }

            return space;
        }

        // BouncyTerrain
        static cpVect[] bouncy_terrain_verts = new cpVect[] {
	new cpVect(537.18f,  23.00f), new cpVect(520.50f,  36.00f), new cpVect(501.53f,  63.00f), new cpVect(496.14f,  76.00f), new cpVect(498.86f,  86.00f), new cpVect(504.00f,  90.51f), new cpVect(508.00f,  91.36f), new cpVect(508.77f,  84.00f), new cpVect(513.00f,  77.73f), new cpVect(519.00f,  74.48f), new cpVect(530.00f,  74.67f), new cpVect(545.00f,  54.65f),
	new cpVect(554.00f,  48.77f), new cpVect(562.00f,  46.39f), new cpVect(568.00f,  45.94f), new cpVect(568.61f,  47.00f), new cpVect(567.94f,  55.00f), new cpVect(571.27f,  64.00f), new cpVect(572.92f,  80.00f), new cpVect(572.00f,  81.39f), new cpVect(563.00f,  79.93f), new cpVect(556.00f,  82.69f), new cpVect(551.49f,  88.00f), new cpVect(549.00f,  95.76f),
	new cpVect(538.00f,  93.40f), new cpVect(530.00f, 102.38f), new cpVect(523.00f, 104.00f), new cpVect(517.00f, 103.02f), new cpVect(516.22f, 109.00f), new cpVect(518.96f, 116.00f), new cpVect(526.00f, 121.15f), new cpVect(534.00f, 116.48f), new cpVect(543.00f, 116.77f), new cpVect(549.28f, 121.00f), new cpVect(554.00f, 130.17f), new cpVect(564.00f, 125.67f),
	new cpVect(575.60f, 129.00f), new cpVect(573.31f, 121.00f), new cpVect(567.77f, 111.00f), new cpVect(575.00f, 106.47f), new cpVect(578.51f, 102.00f), new cpVect(580.25f,  95.00f), new cpVect(577.98f,  87.00f), new cpVect(582.00f,  85.71f), new cpVect(597.00f,  89.46f), new cpVect(604.80f,  95.00f), new cpVect(609.28f, 104.00f), new cpVect(610.55f, 116.00f),
	new cpVect(609.30f, 125.00f), new cpVect(600.80f, 142.00f), new cpVect(597.31f, 155.00f), new cpVect(584.00f, 167.23f), new cpVect(577.86f, 175.00f), new cpVect(583.52f, 184.00f), new cpVect(582.64f, 195.00f), new cpVect(591.00f, 196.56f), new cpVect(597.81f, 201.00f), new cpVect(607.45f, 219.00f), new cpVect(607.51f, 246.00f), new cpVect(600.00f, 275.46f),
	new cpVect(588.00f, 267.81f), new cpVect(579.00f, 264.91f), new cpVect(557.00f, 264.41f), new cpVect(552.98f, 259.00f), new cpVect(548.00f, 246.18f), new cpVect(558.00f, 247.12f), new cpVect(565.98f, 244.00f), new cpVect(571.10f, 237.00f), new cpVect(571.61f, 229.00f), new cpVect(568.25f, 222.00f), new cpVect(562.00f, 217.67f), new cpVect(544.00f, 213.93f),
	new cpVect(536.73f, 214.00f), new cpVect(535.60f, 204.00f), new cpVect(539.69f, 181.00f), new cpVect(542.84f, 171.00f), new cpVect(550.43f, 161.00f), new cpVect(540.00f, 156.27f), new cpVect(536.62f, 152.00f), new cpVect(534.70f, 146.00f), new cpVect(527.00f, 141.88f), new cpVect(518.59f, 152.00f), new cpVect(514.51f, 160.00f), new cpVect(510.33f, 175.00f),
	new cpVect(519.38f, 183.00f), new cpVect(520.52f, 194.00f), new cpVect(516.00f, 201.27f), new cpVect(505.25f, 206.00f), new cpVect(507.57f, 223.00f), new cpVect(519.90f, 260.00f), new cpVect(529.00f, 260.48f), new cpVect(534.00f, 262.94f), new cpVect(538.38f, 268.00f), new cpVect(540.00f, 275.00f), new cpVect(537.06f, 284.00f), new cpVect(530.00f, 289.23f),
	new cpVect(520.00f, 289.23f), new cpVect(513.00f, 284.18f), new cpVect(509.71f, 286.00f), new cpVect(501.69f, 298.00f), new cpVect(501.56f, 305.00f), new cpVect(504.30f, 311.00f), new cpVect(512.00f, 316.43f), new cpVect(521.00f, 316.42f), new cpVect(525.67f, 314.00f), new cpVect(535.00f, 304.98f), new cpVect(562.00f, 294.80f), new cpVect(573.00f, 294.81f),
	new cpVect(587.52f, 304.00f), new cpVect(600.89f, 310.00f), new cpVect(596.96f, 322.00f), new cpVect(603.28f, 327.00f), new cpVect(606.52f, 333.00f), new cpVect(605.38f, 344.00f), new cpVect(597.65f, 352.00f), new cpVect(606.36f, 375.00f), new cpVect(607.16f, 384.00f), new cpVect(603.40f, 393.00f), new cpVect(597.00f, 398.14f), new cpVect(577.00f, 386.15f),
	new cpVect(564.35f, 373.00f), new cpVect(565.21f, 364.00f), new cpVect(562.81f, 350.00f), new cpVect(553.00f, 346.06f), new cpVect(547.48f, 338.00f), new cpVect(547.48f, 330.00f), new cpVect(550.00f, 323.30f), new cpVect(544.00f, 321.53f), new cpVect(537.00f, 322.70f), new cpVect(532.00f, 326.23f), new cpVect(528.89f, 331.00f), new cpVect(527.83f, 338.00f),
	new cpVect(533.02f, 356.00f), new cpVect(542.00f, 360.73f), new cpVect(546.68f, 369.00f), new cpVect(545.38f, 379.00f), new cpVect(537.58f, 386.00f), new cpVect(537.63f, 388.00f), new cpVect(555.00f, 407.47f), new cpVect(563.00f, 413.52f), new cpVect(572.57f, 418.00f), new cpVect(582.72f, 426.00f), new cpVect(578.00f, 431.12f), new cpVect(563.21f, 440.00f),
	new cpVect(558.00f, 449.27f), new cpVect(549.00f, 452.94f), new cpVect(541.00f, 451.38f), new cpVect(536.73f, 448.00f), new cpVect(533.00f, 441.87f), new cpVect(520.00f, 437.96f), new cpVect(514.00f, 429.69f), new cpVect(490.00f, 415.15f), new cpVect(472.89f, 399.00f), new cpVect(472.03f, 398.00f), new cpVect(474.00f, 396.71f), new cpVect(486.00f, 393.61f),
	new cpVect(492.00f, 385.85f), new cpVect(492.00f, 376.15f), new cpVect(489.04f, 371.00f), new cpVect(485.00f, 368.11f), new cpVect(480.00f, 376.27f), new cpVect(472.00f, 379.82f), new cpVect(463.00f, 378.38f), new cpVect(455.08f, 372.00f), new cpVect(446.00f, 377.69f), new cpVect(439.00f, 385.24f), new cpVect(436.61f, 391.00f), new cpVect(437.52f, 404.00f),
	new cpVect(440.00f, 409.53f), new cpVect(463.53f, 433.00f), new cpVect(473.80f, 441.00f), new cpVect(455.00f, 440.30f), new cpVect(443.00f, 436.18f), new cpVect(436.00f, 431.98f), new cpVect(412.00f, 440.92f), new cpVect(397.00f, 442.46f), new cpVect(393.59f, 431.00f), new cpVect(393.71f, 412.00f), new cpVect(400.00f, 395.10f), new cpVect(407.32f, 387.00f),
	new cpVect(408.54f, 380.00f), new cpVect(407.42f, 375.00f), new cpVect(403.97f, 370.00f), new cpVect(399.00f, 366.74f), new cpVect(393.00f, 365.68f), new cpVect(391.23f, 374.00f), new cpVect(387.00f, 380.27f), new cpVect(381.00f, 383.52f), new cpVect(371.56f, 384.00f), new cpVect(364.98f, 401.00f), new cpVect(362.96f, 412.00f), new cpVect(363.63f, 435.00f),
	new cpVect(345.00f, 433.55f), new cpVect(344.52f, 442.00f), new cpVect(342.06f, 447.00f), new cpVect(337.00f, 451.38f), new cpVect(330.00f, 453.00f), new cpVect(325.00f, 452.23f), new cpVect(318.00f, 448.17f), new cpVect(298.00f, 453.70f), new cpVect(284.00f, 451.49f), new cpVect(278.62f, 449.00f), new cpVect(291.47f, 408.00f), new cpVect(291.77f, 398.00f),
	new cpVect(301.00f, 393.83f), new cpVect(305.00f, 393.84f), new cpVect(305.60f, 403.00f), new cpVect(310.00f, 409.47f), new cpVect(318.00f, 413.07f), new cpVect(325.00f, 412.40f), new cpVect(332.31f, 407.00f), new cpVect(335.07f, 400.00f), new cpVect(334.40f, 393.00f), new cpVect(329.00f, 385.69f), new cpVect(319.00f, 382.79f), new cpVect(301.00f, 389.23f),
	new cpVect(289.00f, 389.97f), new cpVect(265.00f, 389.82f), new cpVect(251.00f, 385.85f), new cpVect(245.00f, 389.23f), new cpVect(239.00f, 389.94f), new cpVect(233.00f, 388.38f), new cpVect(226.00f, 382.04f), new cpVect(206.00f, 374.75f), new cpVect(206.00f, 394.00f), new cpVect(204.27f, 402.00f), new cpVect(197.00f, 401.79f), new cpVect(191.00f, 403.49f),
	new cpVect(186.53f, 407.00f), new cpVect(183.60f, 412.00f), new cpVect(183.60f, 422.00f), new cpVect(189.00f, 429.31f), new cpVect(196.00f, 432.07f), new cpVect(203.00f, 431.40f), new cpVect(209.47f, 427.00f), new cpVect(213.00f, 419.72f), new cpVect(220.00f, 420.21f), new cpVect(227.00f, 418.32f), new cpVect(242.00f, 408.41f), new cpVect(258.98f, 409.00f),
	new cpVect(250.00f, 435.43f), new cpVect(239.00f, 438.78f), new cpVect(223.00f, 448.19f), new cpVect(209.00f, 449.70f), new cpVect(205.28f, 456.00f), new cpVect(199.00f, 460.23f), new cpVect(190.00f, 460.52f), new cpVect(182.73f, 456.00f), new cpVect(178.00f, 446.27f), new cpVect(160.00f, 441.42f), new cpVect(148.35f, 435.00f), new cpVect(149.79f, 418.00f),
	new cpVect(157.72f, 401.00f), new cpVect(161.00f, 396.53f), new cpVect(177.00f, 385.00f), new cpVect(180.14f, 380.00f), new cpVect(181.11f, 374.00f), new cpVect(180.00f, 370.52f), new cpVect(170.00f, 371.68f), new cpVect(162.72f, 368.00f), new cpVect(158.48f, 361.00f), new cpVect(159.56f, 349.00f), new cpVect(154.00f, 342.53f), new cpVect(146.00f, 339.85f),
	new cpVect(136.09f, 343.00f), new cpVect(130.64f, 351.00f), new cpVect(131.74f, 362.00f), new cpVect(140.61f, 374.00f), new cpVect(130.68f, 387.00f), new cpVect(120.75f, 409.00f), new cpVect(118.09f, 421.00f), new cpVect(117.92f, 434.00f), new cpVect(100.00f, 432.40f), new cpVect( 87.00f, 427.48f), new cpVect( 81.59f, 423.00f), new cpVect( 73.64f, 409.00f),
	new cpVect( 72.57f, 398.00f), new cpVect( 74.62f, 386.00f), new cpVect( 78.80f, 378.00f), new cpVect( 88.00f, 373.43f), new cpVect( 92.49f, 367.00f), new cpVect( 93.32f, 360.00f), new cpVect( 91.30f, 353.00f), new cpVect(103.00f, 342.67f), new cpVect(109.00f, 343.10f), new cpVect(116.00f, 340.44f), new cpVect(127.33f, 330.00f), new cpVect(143.00f, 327.24f),
	new cpVect(154.30f, 322.00f), new cpVect(145.00f, 318.06f), new cpVect(139.77f, 311.00f), new cpVect(139.48f, 302.00f), new cpVect(144.95f, 293.00f), new cpVect(143.00f, 291.56f), new cpVect(134.00f, 298.21f), new cpVect(118.00f, 300.75f), new cpVect(109.40f, 305.00f), new cpVect( 94.67f, 319.00f), new cpVect( 88.00f, 318.93f), new cpVect( 81.00f, 321.69f),
	new cpVect( 67.24f, 333.00f), new cpVect( 56.68f, 345.00f), new cpVect( 53.00f, 351.40f), new cpVect( 47.34f, 333.00f), new cpVect( 50.71f, 314.00f), new cpVect( 56.57f, 302.00f), new cpVect( 68.00f, 287.96f), new cpVect( 91.00f, 287.24f), new cpVect(110.00f, 282.36f), new cpVect(133.80f, 271.00f), new cpVect(147.34f, 256.00f), new cpVect(156.47f, 251.00f),
	new cpVect(157.26f, 250.00f), new cpVect(154.18f, 242.00f), new cpVect(154.48f, 236.00f), new cpVect(158.72f, 229.00f), new cpVect(166.71f, 224.00f), new cpVect(170.15f, 206.00f), new cpVect(170.19f, 196.00f), new cpVect(167.24f, 188.00f), new cpVect(160.00f, 182.67f), new cpVect(150.00f, 182.66f), new cpVect(143.60f, 187.00f), new cpVect(139.96f, 195.00f),
	new cpVect(139.50f, 207.00f), new cpVect(136.45f, 221.00f), new cpVect(136.52f, 232.00f), new cpVect(133.28f, 238.00f), new cpVect(129.00f, 241.38f), new cpVect(119.00f, 243.07f), new cpVect(115.00f, 246.55f), new cpVect(101.00f, 253.16f), new cpVect( 86.00f, 257.32f), new cpVect( 63.00f, 259.24f), new cpVect( 57.00f, 257.31f), new cpVect( 50.54f, 252.00f),
	new cpVect( 47.59f, 247.00f), new cpVect( 46.30f, 240.00f), new cpVect( 47.58f, 226.00f), new cpVect( 50.00f, 220.57f), new cpVect( 58.00f, 226.41f), new cpVect( 69.00f, 229.17f), new cpVect( 79.00f, 229.08f), new cpVect( 94.50f, 225.00f), new cpVect(100.21f, 231.00f), new cpVect(107.00f, 233.47f), new cpVect(107.48f, 224.00f), new cpVect(109.94f, 219.00f),
	new cpVect(115.00f, 214.62f), new cpVect(122.57f, 212.00f), new cpVect(116.00f, 201.49f), new cpVect(104.00f, 194.57f), new cpVect( 90.00f, 194.04f), new cpVect( 79.00f, 198.21f), new cpVect( 73.00f, 198.87f), new cpVect( 62.68f, 191.00f), new cpVect( 62.58f, 184.00f), new cpVect( 64.42f, 179.00f), new cpVect( 75.00f, 167.70f), new cpVect( 80.39f, 157.00f),
	new cpVect( 68.79f, 140.00f), new cpVect( 61.67f, 126.00f), new cpVect( 61.47f, 117.00f), new cpVect( 64.43f, 109.00f), new cpVect( 63.10f,  96.00f), new cpVect( 56.48f,  82.00f), new cpVect( 48.00f,  73.88f), new cpVect( 43.81f,  66.00f), new cpVect( 43.81f,  56.00f), new cpVect( 50.11f,  46.00f), new cpVect( 59.00f,  41.55f), new cpVect( 71.00f,  42.64f),
	new cpVect( 78.00f,  36.77f), new cpVect( 83.00f,  34.75f), new cpVect( 99.00f,  34.32f), new cpVect(117.00f,  38.92f), new cpVect(133.00f,  55.15f), new cpVect(142.00f,  50.70f), new cpVect(149.74f,  51.00f), new cpVect(143.55f,  68.00f), new cpVect(153.28f,  74.00f), new cpVect(156.23f,  79.00f), new cpVect(157.00f,  84.00f), new cpVect(156.23f,  89.00f),
	new cpVect(153.28f,  94.00f), new cpVect(144.58f,  99.00f), new cpVect(151.52f, 112.00f), new cpVect(151.51f, 124.00f), new cpVect(150.00f, 126.36f), new cpVect(133.00f, 130.25f), new cpVect(126.71f, 125.00f), new cpVect(122.00f, 117.25f), new cpVect(114.00f, 116.23f), new cpVect(107.73f, 112.00f), new cpVect(104.48f, 106.00f), new cpVect(104.32f,  99.00f),
	new cpVect(106.94f,  93.00f), new cpVect(111.24f,  89.00f), new cpVect(111.60f,  85.00f), new cpVect(107.24f,  73.00f), new cpVect(102.00f,  67.57f), new cpVect( 99.79f,  67.00f), new cpVect( 99.23f,  76.00f), new cpVect( 95.00f,  82.27f), new cpVect( 89.00f,  85.52f), new cpVect( 79.84f,  86.00f), new cpVect( 86.73f, 114.00f), new cpVect( 98.00f, 136.73f),
	new cpVect( 99.00f, 137.61f), new cpVect(109.00f, 135.06f), new cpVect(117.00f, 137.94f), new cpVect(122.52f, 146.00f), new cpVect(122.94f, 151.00f), new cpVect(121.00f, 158.58f), new cpVect(134.00f, 160.97f), new cpVect(153.00f, 157.45f), new cpVect(171.30f, 150.00f), new cpVect(169.06f, 142.00f), new cpVect(169.77f, 136.00f), new cpVect(174.00f, 129.73f),
	new cpVect(181.46f, 126.00f), new cpVect(182.22f, 120.00f), new cpVect(182.20f, 111.00f), new cpVect(180.06f, 101.00f), new cpVect(171.28f,  85.00f), new cpVect(171.75f,  80.00f), new cpVect(182.30f,  53.00f), new cpVect(189.47f,  50.00f), new cpVect(190.62f,  38.00f), new cpVect(194.00f,  33.73f), new cpVect(199.00f,  30.77f), new cpVect(208.00f,  30.48f),
	new cpVect(216.00f,  34.94f), new cpVect(224.00f,  31.47f), new cpVect(240.00f,  30.37f), new cpVect(247.00f,  32.51f), new cpVect(249.77f,  35.00f), new cpVect(234.75f,  53.00f), new cpVect(213.81f,  93.00f), new cpVect(212.08f,  99.00f), new cpVect(213.00f, 101.77f), new cpVect(220.00f,  96.77f), new cpVect(229.00f,  96.48f), new cpVect(236.28f, 101.00f),
	new cpVect(240.00f, 107.96f), new cpVect(245.08f, 101.00f), new cpVect(263.00f,  65.32f), new cpVect(277.47f,  48.00f), new cpVect(284.00f,  47.03f), new cpVect(286.94f,  41.00f), new cpVect(292.00f,  36.62f), new cpVect(298.00f,  35.06f), new cpVect(304.00f,  35.77f), new cpVect(314.00f,  43.81f), new cpVect(342.00f,  32.56f), new cpVect(359.00f,  31.32f),
	new cpVect(365.00f,  32.57f), new cpVect(371.00f,  36.38f), new cpVect(379.53f,  48.00f), new cpVect(379.70f,  51.00f), new cpVect(356.00f,  52.19f), new cpVect(347.00f,  54.74f), new cpVect(344.38f,  66.00f), new cpVect(341.00f,  70.27f), new cpVect(335.00f,  73.52f), new cpVect(324.00f,  72.38f), new cpVect(317.00f,  65.75f), new cpVect(313.00f,  67.79f),
	new cpVect(307.57f,  76.00f), new cpVect(315.00f,  78.62f), new cpVect(319.28f,  82.00f), new cpVect(322.23f,  87.00f), new cpVect(323.00f,  94.41f), new cpVect(334.00f,  92.49f), new cpVect(347.00f,  87.47f), new cpVect(349.62f,  80.00f), new cpVect(353.00f,  75.73f), new cpVect(359.00f,  72.48f), new cpVect(366.00f,  72.32f), new cpVect(372.00f,  74.94f),
	new cpVect(377.00f,  81.34f), new cpVect(382.00f,  83.41f), new cpVect(392.00f,  83.40f), new cpVect(399.00f,  79.15f), new cpVect(404.00f,  85.74f), new cpVect(411.00f,  85.06f), new cpVect(417.00f,  86.62f), new cpVect(423.38f,  93.00f), new cpVect(425.05f, 104.00f), new cpVect(438.00f, 110.35f), new cpVect(450.00f, 112.17f), new cpVect(452.62f, 103.00f),
	new cpVect(456.00f,  98.73f), new cpVect(462.00f,  95.48f), new cpVect(472.00f,  95.79f), new cpVect(471.28f,  92.00f), new cpVect(464.00f,  84.62f), new cpVect(445.00f,  80.39f), new cpVect(436.00f,  75.33f), new cpVect(428.00f,  68.46f), new cpVect(419.00f,  68.52f), new cpVect(413.00f,  65.27f), new cpVect(408.48f,  58.00f), new cpVect(409.87f,  46.00f),
	new cpVect(404.42f,  39.00f), new cpVect(408.00f,  33.88f), new cpVect(415.00f,  29.31f), new cpVect(429.00f,  26.45f), new cpVect(455.00f,  28.77f), new cpVect(470.00f,  33.81f), new cpVect(482.00f,  42.16f), new cpVect(494.00f,  46.85f), new cpVect(499.65f,  36.00f), new cpVect(513.00f,  25.95f), new cpVect(529.00f,  22.42f), new cpVect(537.18f,  23.00f), 
};

        static public cpSpace BouncyTerrainCircles_500(cpSpace space)
        {
            //cpSpace space = BENCH_SPACE_NEW();
            SetSubTitle("BouncyTerrainCircles 500");
            space.SetIterations(10);

            cpVect offset = new cpVect(-320, -240);
            for (int i = 0; i < (bouncy_terrain_verts.Length - 1); i++)
            {
                cpVect a = bouncy_terrain_verts[i], b = bouncy_terrain_verts[i + 1];
                cpShape shape = space.AddShape(new cpSegmentShape(space.GetStaticBody(), cpVect.cpvadd(a, offset), cpVect.cpvadd(b, offset), 0.0f));
                shape.SetElasticity(1.0f);
            }

            for (int i = 0; i < 500; i++)
            {
                float radius = 5.0f;
                float mass = radius * radius;
                cpBody body = space.AddBody(new cpBody(mass, cp.MomentForCircle(mass, 0.0f, radius, cpVect.Zero)));
                body.SetPosition(cpVect.cpvadd(cpVect.cpvmult(cp.frand_unit_circle(), 130.0f), cpVect.Zero));
                body.SetVelocity(cpVect.cpvmult(cp.frand_unit_circle(), 50.0f));

                cpShape shape = space.AddShape(new cpCircleShape(body, radius, cpVect.Zero));
                shape.SetElasticity(1.0f);
            }

            return space;
        }

        static public cpSpace BouncyTerrainHexagons_500(cpSpace space)
        {
            SetSubTitle("BouncyTerrainHexagons 500");
            //cpSpace space = BENCH_SPACE_NEW();
            space.SetIterations(10);

            cpVect offset = new cpVect(-320, -240);
            for (int i = 0; i < (bouncy_terrain_verts.Length - 1); i++)
            {
                cpVect a = bouncy_terrain_verts[i], b = bouncy_terrain_verts[i + 1];
                cpShape shape = space.AddShape(new cpSegmentShape(space.GetStaticBody(), cpVect.cpvadd(a, offset), cpVect.cpvadd(b, offset), 0.0f));
                shape.SetElasticity(1.0f);
            }

            float radius = 5.0f;
            cpVect[] hexagon = new cpVect[6];
            for (int i = 0; i < 6; i++)
            {
                float angle = -(float)Math.PI * 2.0f * i / 6.0f;
                hexagon[i] = cpVect.cpvmult(cpVect.cpv(cp.cpfcos(angle), cp.cpfsin(angle)), radius - bevel);
            }

            for (int i = 0; i < 500; i++)
            {
                float mass = radius * radius;
                cpBody body = space.AddBody(new cpBody(mass, cp.MomentForPoly(mass, 6, hexagon, cpVect.Zero, 0.0f)));
                body.SetPosition(cpVect.cpvadd(cpVect.cpvmult(cp.frand_unit_circle(), 130.0f), cpVect.Zero));
                body.SetVelocity(cpVect.cpvmult(cp.frand_unit_circle(), 50.0f));

                cpShape shape = space.AddShape(new cpPolyShape(body, 6, hexagon, cpTransform.Identity, bevel));
                shape.SetElasticity(1.0f);
            }

            return space;
        }

        // No collisions

        static bool NoCollide_begin(cpArbiter arb, cpSpace space, object data)
        {
            //abort();
            return true;
        }

        static cpSpace NoCollide(cpSpace space)
        {
            space.SetIterations(10);

            var handler = space.AddCollisionHandler(2, 2);
            //, (a, s, o) => NoCollide_begin(a, s, o), null, null, null);
            handler.beginFunc = NoCollide_begin;
            float radius = 4.5f;
            var staticBody = space.GetStaticBody();

            space.AddShape(new cpSegmentShape(staticBody, new cpVect(-330 - radius, -250 - radius), new cpVect(330 + radius, -250 - radius), 0.0f)).SetElasticity(1.0f);
            space.AddShape(new cpSegmentShape(staticBody, new cpVect(330 + radius, 250 + radius), new cpVect(330 + radius, -250 - radius), 0.0f)).SetElasticity(1.0f);
            space.AddShape(new cpSegmentShape(staticBody, new cpVect(330 + radius, 250 + radius), new cpVect(-330 - radius, 250 + radius), 0.0f)).SetElasticity(1.0f);
            space.AddShape(new cpSegmentShape(staticBody, new cpVect(-330 - radius, -250 - radius), new cpVect(-330 - radius, 250 + radius), 0.0f)).SetElasticity(1.0f);

            for (int x = -320; x <= 320; x += 20)
            {
                for (int y = -240; y <= 240; y += 20)
                    space.AddShape(new cpCircleShape(staticBody, radius, new cpVect(x, y)));
            }

            for (int y = 10 - 240; y <= 240; y += 40)
            {
                float mass = 7.0f;
                cpBody body = space.AddBody(new cpBody(mass, cp.MomentForCircle(mass, 0.0f, radius, cpVect.Zero)));
                body.SetPosition(new cpVect(-320.0f, y));
                body.SetVelocity(new cpVect(100.0f, 0.0f));

                cpShape shape = space.AddShape(new cpCircleShape(body, radius, cpVect.Zero));
                shape.SetElasticity(1.0f);
                shape.SetCollisionType(2);

            }

            for (int x = 30 - 320; x <= 320; x += 40)
            {
                float mass = 7.0f;
                cpBody body = space.AddBody(new cpBody(mass, cp.MomentForCircle(mass, 0.0f, radius, cpVect.Zero)));

                body.SetPosition(new cpVect(x, -240.0f));
                body.SetVelocity(new cpVect(0.0f, 100.0f));

                cpShape shape = space.AddShape(new cpCircleShape(body, radius, cpVect.Zero));
                shape.SetElasticity(1.0f);
                shape.SetCollisionType(2);
            }
            return space;
        }

        public override void Update(float dt)
        {
            base.Update(dt);
            space.Step(dt);
        }
    }
}
