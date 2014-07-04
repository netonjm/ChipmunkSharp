using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace ChipmunkExample
{
    public class Settings
    {
        public Settings()
        {
            hz = 60.0f;
            velocityIterations = 8;// 10;
            positionIterations = 3;// 8;
            drawShapes = 1;
            drawJoints = 1;
            enableWarmStarting = 1;
            enableContinuous = 1;
        }

        public float hz;
        public int velocityIterations;
        public int positionIterations;
        public uint drawShapes;
        public uint drawJoints;
        public uint drawAABBs;
        public uint drawPairs;
        public uint drawContactPoints;
        public uint drawContactNormals;
        public uint drawContactForces;
        public uint drawFrictionForces;
        public uint drawCOMs;
        public uint drawStats;
        public uint enableWarmStarting;
        public uint enableContinuous;
        public uint pause;
        public uint singleStep;
    }
}
