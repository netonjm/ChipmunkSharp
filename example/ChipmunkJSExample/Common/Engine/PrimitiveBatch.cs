using ChipmunkSharp;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace ChipmunkExample
{


    internal static class cpVecHelper
    {

        internal static Vector3 ToVector3(this Vector2 v)
        {
            return new Vector3(v.X, v.Y, 0f);
        }

        internal static cpVect ToCpVect(this Vector2 vec)
        {
            return new cpVect(vec.X, vec.Y);
        }

        internal static Vector2 ToVector2(this cpVect vec)
        {
            return new Vector2(vec.x, vec.y);
        }

        //internal static Vector3 ToVector3(this Vector2 vec)
        //{
        //    return new Vector2(vec.X, vec.Y);
        //}

        internal static Vector3 ToVector3(this cpVect vec)
        {
            return vec.ToVector2().ToVector3();
            //return new Vector2(vec.x, vec.y).ToVector3();
        }

        //public static CCVector2 ToCCVector2(this cpVect vec)
        //{
        //    return new CCVector2(vec.X, vec.Y);
        //}

        internal static Color ToColor(this cpColor color)
        {
            return new Color(color.r, color.g, color.b);
        }

        internal static Color ToCCColor4B(this cpColor color)
        {
            return new Color(color.r, color.g, color.b, 255);
        }
    }

    // PrimitiveBatch is a class that handles efficient rendering automatically for its
    // users, in a similar way to SpriteBatch. PrimitiveBatch can render lines, points,
    // and triangles to the screen. In this sample, it is used to draw a spacewars
    // retro scene.
    public class PrimitiveBatch : cpDraw, IDisposable
    {



        #region Constants and Fields

        const int SEGMENTS = 50;
        // this constant controls how large the vertices buffer is. Larger buffers will
        // require flushing less often, which can increase performance. However, having
        // buffer that is unnecessarily large will waste memory.
        const int DefaultBufferSize = 5000000;

        // a block of vertices that calling AddVertex will fill. Flush will draw using
        // this array, and will determine how many primitives to draw from
        // positionInBuffer.
        VertexPositionColor[] vertices = new VertexPositionColor[DefaultBufferSize];

        VertexPositionColor[] _vertsLines = new VertexPositionColor[DefaultBufferSize];
        VertexPositionColor[] _vertsFill = new VertexPositionColor[DefaultBufferSize];

        // keeps track of how many vertices have been added. this value increases until
        // we run out of space in the buffer, at which time Flush is automatically
        // called.
        int positionInBuffer = 0;

        // a basic effect, which contains the shaders that we will use to draw our
        // primitives.
        BasicEffect basicEffect;

        // the device that we will issue draw calls to.
        GraphicsDevice device;

        // this value is set by Begin, and is the type of primitives that we are
        // drawing.
        PrimitiveType primitiveType;

        // how many verts does each of these primitives take up? points are 1,
        // lines are 2, and triangles are 3.
        int numVertsPerPrimitive;

        // hasBegun is flipped to true once Begin is called, and is used to make
        // sure users don't call End before Begin is called.
        bool hasBegun = false;

        bool isDisposed = false;

        public float LineWidth { get; set; }

        public cpColor DrawColor { get; set; }

        #endregion

        // the constructor creates a new PrimitiveBatch and sets up all of the internals
        // that PrimitiveBatch will need.
        public PrimitiveBatch(GraphicsDevice graphicsDevice)
        {
            if (graphicsDevice == null)
            {
                throw new ArgumentNullException("graphicsDevice");
            }

            LineWidth = 1;
            device = graphicsDevice;

            // set up a new basic effect, and enable vertex colors.
            basicEffect = new BasicEffect(graphicsDevice);
            basicEffect.VertexColorEnabled = true;

            // projection uses CreateOrthographicOffCenter to create 2d projection
            // matrix with 0,0 in the upper left.
            basicEffect.Projection = Matrix.CreateOrthographicOffCenter
                (0, graphicsDevice.Viewport.Width,
                graphicsDevice.Viewport.Height, 0,
                0, 1);
            this.basicEffect.World = Matrix.Identity;
            this.basicEffect.View = Matrix.CreateLookAt(Vector3.Zero, Vector3.Forward,
                Vector3.Up);
        }

        public void Dispose()
        {
            this.Dispose(true);
            GC.SuppressFinalize(this);
        }

        protected virtual void Dispose(bool disposing)
        {
            if (disposing && !isDisposed)
            {
                if (basicEffect != null)
                    basicEffect.Dispose();

                isDisposed = true;
            }
        }

        // Begin is called to tell the PrimitiveBatch what kind of primitives will be
        // drawn, and to prepare the graphics card to render those primitives.
        public void Begin(PrimitiveType primitiveType)
        {
            if (hasBegun)
            {
                throw new InvalidOperationException
                    ("End must be called before Begin can be called again.");
            }

            // these three types reuse vertices, so we can't flush properly without more
            // complex logic. Since that's a bit too complicated for this sample, we'll
            // simply disallow them.
            if (primitiveType == PrimitiveType.LineStrip ||
                primitiveType == PrimitiveType.TriangleStrip)
            {
                throw new NotSupportedException
                    ("The specified primitiveType is not supported by PrimitiveBatch.");
            }

            this.primitiveType = primitiveType;

            // how many verts will each of these primitives require?
            this.numVertsPerPrimitive = NumVertsPerPrimitive(primitiveType);

            //tell our basic effect to begin.
            basicEffect.CurrentTechnique.Passes[0].Apply();

            // flip the error checking boolean. It's now ok to call AddVertex, Flush,
            // and End.
            hasBegun = true;
        }



        public override void DrawPolygon(List<cpVect> vertices, int vertexCount, cpColor color)
        {
            var actualColor = color.ToColor();

            for (int i = 0; i < vertexCount - 1; i++)
            {
                _vertsLines[_lineCount * 2].Position = vertices[i].ToVector3();
                _vertsLines[_lineCount * 2].Color = actualColor;
                _vertsLines[_lineCount * 2 + 1].Position = vertices[i + 1].ToVector3();
                _vertsLines[_lineCount * 2 + 1].Color = actualColor;
                _lineCount++;
            }

            _vertsLines[_lineCount * 2].Position = vertices[vertexCount - 1].ToVector3();
            _vertsLines[_lineCount * 2].Color = actualColor;
            _vertsLines[_lineCount * 2 + 1].Position = vertices[0].ToVector3();
            _vertsLines[_lineCount * 2 + 1].Color = actualColor;
            _lineCount++;

        }

        public override void DrawSolidPolygon(List<cpVect> vertices, int count, cpColor color)
        {
            DrawSolidPolygon(vertices, count, color, true);
        }

        public void DrawSolidPolygon(List<cpVect> vertices, int count, cpColor color, bool outline)
        {

            var actualColor = color.ToColor();

            if (count == 2)
            {
                DrawPolygon(vertices, count, color);
                return;
            }

            Color colorFill = actualColor * (outline ? 0.5f : 1.0f);

            for (int i = 1; i < count - 1; i++)
            {
                _vertsFill[_fillCount * 3].Position = vertices[0].ToVector3(); // new Vector3(, 0.0f);
                _vertsFill[_fillCount * 3].Color = colorFill;

                _vertsFill[_fillCount * 3 + 1].Position = vertices[i].ToVector3(); // new Vector3, 0.0f);
                _vertsFill[_fillCount * 3 + 1].Color = colorFill;

                _vertsFill[_fillCount * 3 + 2].Position = vertices[i + 1].ToVector3(); // new Vector3(vertices[i + 1], 0.0f);
                _vertsFill[_fillCount * 3 + 2].Color = colorFill;

                _fillCount++;
            }

            if (outline)
            {
                DrawPolygon(vertices, count, color);
            }
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="center"></param>
        /// <param name="radius"></param>
        /// <param name="angle">The amount of the circle to draw, in radiians</param>
        /// <param name="segments"></param>
        /// <param name="drawLineToCenter"></param>
        /// <param name="color"></param>
        public void DrawCircle(cpVect center, float radius, float angle, int segments, bool drawLineToCenter, float scaleX = 1.0f, float scaleY = 1.0f)
        {
            DrawCircle(center, radius, angle, segments, drawLineToCenter, DrawColor, scaleX, scaleY);
        }

        public void DrawSolidCircle(cpVect center, float radius, float angle, int segments, float scaleX = 1.0f, float scaleY = 1.0f)
        {
            DrawSolidCircle(center, radius, angle, segments, DrawColor, scaleX, scaleY);
        }

        /// <summary>
        /// draws a poligon given a pointer to CCPoint coordiantes and the number of vertices measured in points.
        /// The polygon can be closed or open
        /// </summary>
        public void DrawPoly(cpVect[] vertices, cpColor color, bool closePolygon = false)
        {
            DrawPoly(vertices, vertices.Length, closePolygon, false, color);
        }

        /// <summary>
        /// draws a polygon given a pointer to CCPoint coordiantes and the number of vertices measured in points.
        /// The polygon can be closed or open and optionally filled with current GL color
        /// </summary>
        //public void DrawPoly(cpVect[] vertices, int numOfVertices, bool closePolygon, bool fill, cpColor color)
        //{

        //    Color col = color.ToColor();

        //    if (fill)
        //    {
        //        for (int i = 1; i < numOfVertices - 1; i++)
        //        {
        //            AddVertex(new Vector2(vertices[0].x, vertices[0].y), col);
        //            AddVertex(new Vector2(vertices[i].x, vertices[i].y), col);
        //            AddVertex(new Vector2(vertices[i + 1].x, vertices[i + 1].y), col);
        //        }
        //    }
        //    else
        //    {
        //        for (int i = 0; i < numOfVertices - 1; i++)
        //        {
        //            DrawLine(vertices[i], vertices[i + 1], col);
        //        }

        //        if (closePolygon)
        //        {
        //            DrawLine(vertices[numOfVertices - 1], vertices[0], col);
        //        }
        //    }
        //}

        public void DrawCircle(cpVect center, float radius, float angle, int segments, bool drawLineToCenter, cpColor color, float scaleX = 1.0f, float scaleY = 1.0f)
        {
            float increment = MathHelper.Pi * 2.0f / segments;
            double theta = radius;

            var vertices = new cpVect[segments * 2];

            for (int i = 0, s = 0; i < segments; i++, s += 2)
            {
                vertices[s] = center + new cpVect((float)Math.Cos(theta) * scaleX, (float)Math.Sin(theta) * scaleY) * radius;
                vertices[s + 1] = center + new cpVect((float)Math.Cos(theta + increment) * scaleX, (float)Math.Sin(theta + increment) * scaleY) * radius;

                theta += increment;
            }

            DrawPoly(vertices, color);

            if (drawLineToCenter)
            {
                DrawLine(center, vertices[vertices.Length - 1], color);
            }

            //var actualColor = color.ToColor();

            //int segments = 16;
            //double increment = Math.PI * 2.0 / (double)segments;
            //double theta = 0.0;

            //for (int i = 0; i < segments; i++)
            //{

            //    Vector2 v1 = center.ToVector2() + radius * new Vector2((float)Math.Cos(theta), (float)Math.Sin(theta));
            //    Vector2 v2 = center.ToVector2() + radius * new Vector2((float)Math.Cos(theta + increment), (float)Math.Sin(theta + increment));

            //    _vertsLines[_lineCount * 2].Position = new Vector3(v1, 0.0f);
            //    _vertsLines[_lineCount * 2].Color = actualColor;
            //    _vertsLines[_lineCount * 2 + 1].Position = new Vector3(v2, 0.0f);
            //    _vertsLines[_lineCount * 2 + 1].Color = actualColor;
            //    _lineCount++;

            //    theta += increment;
            //}
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="center"></param>
        /// <param name="radius"></param>
        /// <param name="angle">The amount of the circle to draw, in radiians</param>
        /// <param name="segments"></param>
        /// <param name="color"></param>
        public void DrawSolidCircle(cpVect center, float radius, float angle, int segments, cpColor color, float scaleX = 1.0f, float scaleY = 1.0f)
        {
            float increment = MathHelper.Pi * 2.0f / segments;
            double theta = angle;

            var vertices = new cpVect[segments * 2];

            for (int i = 0, s = 0; i < segments; i++, s += 2)
            {
                vertices[s] = center + new cpVect((float)Math.Cos(theta) * scaleX, (float)Math.Sin(theta) * scaleY) * radius;
                vertices[s + 1] = center + new cpVect((float)Math.Cos(theta + increment) * scaleX, (float)Math.Sin(theta + increment) * scaleY) * radius;

                theta += increment;
            }

            DrawSolidPoly(vertices, color);
        }

        public void DrawSolidPoly(cpVect[] vertices, cpColor color)
        {
            DrawSolidPoly(vertices, vertices.Length, color, false);
        }

        public void DrawSolidPoly(cpVect[] vertices, int count, cpColor color, bool outline)
        {

            Color col = color.ToColor();

            if (count == 2)
            {
                DrawPoly(vertices, count, false, color);
                return;
            }

            var colorFill = col * (outline ? 0.5f : 1.0f);

            for (int i = 1; i < count - 1; i++)
            {
                AddVertex(new Vector2(vertices[0].x, vertices[0].y), colorFill);
                AddVertex(new Vector2(vertices[i].x, vertices[i].y), colorFill);
                AddVertex(new Vector2(vertices[i + 1].x, vertices[i + 1].y), colorFill);
            }

            if (outline)
            {
                DrawPoly(vertices, count, true, color);
            }
        }

        /// <summary>
        /// draws a poligon given a pointer to CCPoint coordiantes and the number of vertices measured in points.
        /// The polygon can be closed or open
        /// </summary>
        public void DrawPoly(cpVect[] vertices, int numOfVertices, bool closePolygon, cpColor color)
        {
            DrawPoly(vertices, numOfVertices, closePolygon, false, color);
        }


        public void DrawArc(Rectangle rect, int startAngle, int sweepAngle)
        {
            DrawEllipticalArc(rect, startAngle, sweepAngle, false, DrawColor);
        }

        public void DrawArc(Rectangle rect, int startAngle, int sweepAngle, cpColor color)
        {
            DrawEllipticalArc(rect, startAngle, sweepAngle, false, color);
        }

        public void DrawArc(int x, int y, int width, int height, int startAngle, int sweepAngle)
        {
            DrawEllipticalArc(x, y, width, height, startAngle, sweepAngle, false, DrawColor);

        }

        public void DrawArc(int x, int y, int width, int height, int startAngle, int sweepAngle, cpColor color)
        {
            DrawEllipticalArc(x, y, width, height, startAngle, sweepAngle, false, color);

        }

        internal void DrawEllipticalArc(Rectangle arcRect, double lambda1, double lambda2,
                                           bool isPieSlice, cpColor color)
        {
#if MAISONABE
            DrawEllipticalArc(arcRect.Origin.X + arcRect.Size.Width / 2,
                              arcRect.Origin.Y + arcRect.Size.Height / 2,
                              arcRect.Size.Width / 2,
                              arcRect.Size.Height / 2, 0, lambda1, lambda2, isPieSlice, color
                              );
#else
            make_arcs(
                      arcRect.X, arcRect.Y, arcRect.Width, arcRect.Height,
                      (float)lambda1, (float)lambda2,
                      false, true, isPieSlice, color);
#endif
        }


        internal void DrawEllipticalArc(float x, float y, float width, float height, double lambda1, double lambda2,
                                               bool isPieSlice, cpColor color)
        {
#if MAISONABE
            DrawEllipticalArc(x + width / 2,
                              y + height / 2,
                              width / 2,
                              height / 2, 0, lambda1, lambda2, isPieSlice, color
                              );
#else
            make_arcs(
                      x, y, width, height,
                      (float)lambda1, (float)lambda2,
                      false, true, isPieSlice, color);
#endif
        }

#if MAISONABE
        
        /** Build an elliptical arc from its canonical geometrical elements.
        * @param cx abscissa of the center of the ellipse
        * @param cy ordinate of the center of the ellipse
        * @param a semi-major axis
        * @param b semi-minor axis
        * @param theta orientation of the major axis with respect to the x axis
        * @param lambda1 start angle of the arc
        * @param lambda2 end angle of the arc
        * @param isPieSlice if true, the lines between the center of the ellipse
        * and the endpoints are part of the shape (it is pie slice like)
        */
        internal static void DrawEllipticalArc(double centerX, double centerY, double axisA, double axisB,
                                        double thetaOrientation, double lambda1, double lambda2,
                                               bool isPie, CCColor4B color)
        {
            cx = centerX;
            cy = centerY;
            a = axisA;
            b = axisB;
            theta = thetaOrientation;
            isPieSlice = isPie;
            
            // Angles in radians
            lambda1 = lambda1 * Math.PI / 180;
            lambda2 = lambda2 * Math.PI / 180;
            
            // Handle negative sweep angles
            if (lambda2 < 0)
            {
                var temp = lambda1;
                lambda1 += lambda2;
                lambda2 = temp;
                
            }
            else
                lambda2 += lambda1;
            
            eta1 = Math.Atan2(Math.Sin(lambda1) / b,
                              Math.Cos(lambda1) / a);
            eta2 = Math.Atan2(Math.Sin(lambda2) / b,
                              Math.Cos(lambda2) / a);
            cosTheta = Math.Cos(theta);
            sinTheta = Math.Sin(theta);
            
            // make sure we have eta1 <= eta2 <= eta1 + 2 PI
            eta2 -= twoPi * Math.Floor((eta2 - eta1) / twoPi);
            
            // the preceding correction fails if we have exactly et2 - eta1 = 2 PI
            // it reduces the interval to zero length
            if ((lambda2 - lambda1 > Math.PI) && (eta2 - eta1 < Math.PI))
            {
                eta2 += 2 * Math.PI;
            }
            
            computeFocii();
            
            // NOTE: Max degrees handled by the routine is 3 
            drawEllipticalArcToContext(3, 0, color);
        }
        
        /** Compute the locations of the focii. */
        private static void computeFocii()
        {
            
            double d = Math.Sqrt(a * a - b * b);
            double dx = d * cosTheta;
            double dy = d * sinTheta;
            
            xF1 = cx - dx;
            yF1 = cy - dy;
            xF2 = cx + dx;
            yF2 = cy + dy;
            
        }
        
        /** Compute the value of a rational function.
        * This method handles rational functions where the numerator is
        * quadratic and the denominator is linear
        * @param x absissa for which the value should be computed
        * @param c coefficients array of the rational function
        */
        private static double rationalFunction(double x, double[] c)
        {
            return (x * (x * c[0] + c[1]) + c[2]) / (x + c[3]);
        }
        
        /** Estimate the approximation error for a sub-arc of the instance.
        * @param degree degree of the Bézier curve to use (1, 2 or 3)
        * @param tA start angle of the sub-arc
        * @param tB end angle of the sub-arc
        * @return upper bound of the approximation error between the Bézier
        * curve and the real ellipse
        */
        private static double estimateError(int degree, double etaA, double etaB)
        {
            
            double eta = 0.5 * (etaA + etaB);
            
            if (degree < 2)
            {
                
                // start point
                double aCosEtaA = a * Math.Cos(etaA);
                double bSinEtaA = b * Math.Sin(etaA);
                double xA = cx + aCosEtaA * cosTheta - bSinEtaA * sinTheta;
                double yA = cy + aCosEtaA * sinTheta + bSinEtaA * cosTheta;
                
                // end point
                double aCosEtaB = a * Math.Cos(etaB);
                double bSinEtaB = b * Math.Sin(etaB);
                double xB = cx + aCosEtaB * cosTheta - bSinEtaB * sinTheta;
                double yB = cy + aCosEtaB * sinTheta + bSinEtaB * cosTheta;
                
                // maximal error point
                double aCosEta = a * Math.Cos(eta);
                double bSinEta = b * Math.Sin(eta);
                double x = cx + aCosEta * cosTheta - bSinEta * sinTheta;
                double y = cy + aCosEta * sinTheta + bSinEta * cosTheta;
                
                double dx = xB - xA;
                double dy = yB - yA;
                
                return Math.Abs(x * dy - y * dx + xB * yA - xA * yB)
                    / Math.Sqrt(dx * dx + dy * dy);
                
            }
            else
            {
                
                double x = b / a;
                double dEta = etaB - etaA;
                double cos2 = Math.Cos(2 * eta);
                double cos4 = Math.Cos(4 * eta);
                double cos6 = Math.Cos(6 * eta);
                
                // select the right coeficients set according to degree and b/a
                double[][][] coeffs;
                double[] safety;
                if (degree == 2)
                {
                    coeffs = (x < 0.25) ? coeffs2Low : coeffs2High;
                    safety = safety2;
                }
                else
                {
                    coeffs = (x < 0.25) ? coeffs3Low : coeffs3High;
                    safety = safety3;
                }
                
                double c0 = rationalFunction(x, coeffs[0][0])
                    + cos2 * rationalFunction(x, coeffs[0][1])
                        + cos4 * rationalFunction(x, coeffs[0][2])
                        + cos6 * rationalFunction(x, coeffs[0][3]);
                
                double c1 = rationalFunction(x, coeffs[1][0])
                    + cos2 * rationalFunction(x, coeffs[1][1])
                        + cos4 * rationalFunction(x, coeffs[1][2])
                        + cos6 * rationalFunction(x, coeffs[1][3]);
                
                return rationalFunction(x, safety) * a * Math.Exp(c0 + c1 * dEta);
                
            }
            
        }
        
        /** Build an approximation of the instance outline.
        * @param degree degree of the Bézier curve to use
        * @param threshold acceptable error
        */
        public static void drawEllipticalArcToContext(int degree, double threshold, CCColor4B color)
        {
            
            // find the number of Bézier curves needed
            bool found = false;
            int n = 1;
            while ((!found) && (n < 1024))
            {
                double dEta2 = (eta2 - eta1) / n;
                if (dEta2 <= 0.5 * Math.PI)
                {
                    double etaB2 = eta1;
                    found = true;
                    for (int i = 0; found && (i < n); ++i)
                    {
                        double etaA = etaB2;
                        etaB2 += dEta2;
                        found = (estimateError(degree, etaA, etaB2) <= threshold);
                    }
                }
                n = n << 1;
            }
            
            double dEta = (eta2 - eta1) / n;
            double etaB = eta1;
            
            double cosEtaB = Math.Cos(etaB);
            double sinEtaB = Math.Sin(etaB);
            double aCosEtaB = a * cosEtaB;
            double bSinEtaB = b * sinEtaB;
            double aSinEtaB = a * sinEtaB;
            double bCosEtaB = b * cosEtaB;
            double xB = cx + aCosEtaB * cosTheta - bSinEtaB * sinTheta;
            double yB = cy + aCosEtaB * sinTheta + bSinEtaB * cosTheta;
            double xBDot = -aSinEtaB * cosTheta - bCosEtaB * sinTheta;
            double yBDot = -aSinEtaB * sinTheta + bCosEtaB * cosTheta;

            CCPoint startPoint = CCPoint.Zero;
            CCPoint piePoint = CCPoint.Zero;

            if (isPieSlice)
            {

                startPoint.X = (float)cx;
                startPoint.Y = (float)cy;
                piePoint.X = (float)cx;
                piePoint.Y = (float)cy;

            }
            else
            {
                startPoint.X = (float)xB;
                startPoint.Y = (float)yB;
            }
            
            double t = Math.Tan(0.5 * dEta);
            double alpha = Math.Sin(dEta) * (Math.Sqrt(4 + 3 * t * t) - 1) / 3;
 
            CCPoint destinationPoint = CCPoint.Zero;
            CCPoint controlPoint1 = CCPoint.Zero;
            CCPoint controlPoint2 = CCPoint.Zero;

            for (int i = 0; i < n; ++i)
            {
                
                //double etaA = etaB;
                double xA = xB;
                double yA = yB;
                double xADot = xBDot;
                double yADot = yBDot;
                
                etaB += dEta;
                cosEtaB = Math.Cos(etaB);
                sinEtaB = Math.Sin(etaB);
                aCosEtaB = a * cosEtaB;
                bSinEtaB = b * sinEtaB;
                aSinEtaB = a * sinEtaB;
                bCosEtaB = b * cosEtaB;
                xB = cx + aCosEtaB * cosTheta - bSinEtaB * sinTheta;
                yB = cy + aCosEtaB * sinTheta + bSinEtaB * cosTheta;
                xBDot = -aSinEtaB * cosTheta - bCosEtaB * sinTheta;
                yBDot = -aSinEtaB * sinTheta + bCosEtaB * cosTheta;
                
                destinationPoint.X = (float)xB;
                destinationPoint.Y = (float)yB;

                if (degree == 1)
                {

                    DrawLine(startPoint, destinationPoint, color);
                }
                else if (degree == 2)
                {
                    double k = (yBDot * (xB - xA) - xBDot * (yB - yA))
                        / (xADot * yBDot - yADot * xBDot);

                    controlPoint1.X = (float)(xA + k * xADot);
                    controlPoint1.Y = (float)(yA + k * yADot);

                    DrawQuadBezier(startPoint, controlPoint1, destinationPoint, SEGMENTS, color);
                }
                else
                {
                    controlPoint1.X = (float)(xA + alpha * xADot);
                    controlPoint1.Y = (float)(yA + alpha * yADot);

                    controlPoint2.X = (float)(xB - alpha * xBDot);
                    controlPoint2.Y = (float)(yB - alpha * yBDot);


                    DrawCubicBezier(startPoint, controlPoint1, controlPoint2, destinationPoint, SEGMENTS, color); 

                }

                startPoint.X = (float)xB;
                startPoint.Y = (float)yB;

            }
            
            if (isPieSlice)
            {

                DrawLine(piePoint, destinationPoint, color);
            }
            
        }
#endif

        static cpVect startPoint = cpVect.ZERO;
        static cpVect destinationPoint = cpVect.ZERO;
        static cpVect controlPoint1 = cpVect.ZERO;
        static cpVect controlPoint2 = cpVect.ZERO;


        /*
         * Based on the algorithm described in
         *      http://www.stillhq.com/ctpfaq/2002/03/c1088.html#AEN1212
         */
        void
            make_arc(bool start, float x, float y, float width,
                     float height, float startAngle, float endAngle, bool antialiasing, bool isPieSlice, cpColor color)
        {
            float delta, bcp;
            double sin_alpha, sin_beta, cos_alpha, cos_beta;
            float PI = (float)Math.PI;

            float rx = width / 2;
            float ry = height / 2;

            /* center */
            float cx = x + rx;
            float cy = y + ry;

            /* angles in radians */
            float alpha = startAngle * PI / 180;
            float beta = endAngle * PI / 180;

            /* adjust angles for ellipses */
            alpha = (float)Math.Atan2(rx * Math.Sin(alpha), ry * Math.Cos(alpha));
            beta = (float)Math.Atan2(rx * Math.Sin(beta), ry * Math.Cos(beta));

            if (Math.Abs(beta - alpha) > PI)
            {
                if (beta > alpha)
                    beta -= 2 * PI;
                else
                    alpha -= 2 * PI;
            }

            delta = beta - alpha;
            bcp = (float)(4.0 / 3.0 * (1 - Math.Cos(delta / 2)) / Math.Sin(delta / 2));

            sin_alpha = Math.Sin(alpha);
            sin_beta = Math.Sin(beta);
            cos_alpha = Math.Cos(alpha);
            cos_beta = Math.Cos(beta);

            /* don't move to starting point if we're continuing an existing curve */
            if (start)
            {
                /* starting point */
                double sx = cx + rx * cos_alpha;
                double sy = cy + ry * sin_alpha;
                if (isPieSlice)
                {
                    destinationPoint.x = (float)sx;
                    destinationPoint.y = (float)sy;

                    DrawLine(startPoint, destinationPoint, color);
                }

                startPoint.x = (float)sx;
                startPoint.y = (float)sy;
            }

            destinationPoint.x = cx + rx * (float)cos_beta;
            destinationPoint.y = cy + ry * (float)sin_beta;

            controlPoint1.x = cx + rx * (float)(cos_alpha - bcp * sin_alpha);
            controlPoint1.y = cy + ry * (float)(sin_alpha + bcp * cos_alpha);

            controlPoint2.x = cx + rx * (float)(cos_beta + bcp * sin_beta);
            controlPoint2.y = cy + ry * (float)(sin_beta - bcp * cos_beta);


            DrawCubicBezier(startPoint, controlPoint1, controlPoint2, destinationPoint, SEGMENTS, color);

            startPoint.x = destinationPoint.x;
            startPoint.y = destinationPoint.y;
        }

        public void DrawCubicBezier(cpVect origin, cpVect control1, cpVect control2, cpVect destination, int segments)
        {
            DrawCubicBezier(origin, control1, control2, destination, segments, DrawColor);
        }

        /// <summary>
        /// draws a cubic bezier path
        /// @since v0.8
        /// </summary>
        public void DrawCubicBezier(cpVect origin, cpVect control1, cpVect control2, cpVect destination, int segments, cpColor color)
        {

            float t = 0;
            float increment = 1.0f / segments;

            var vertices = new cpVect[segments];

            vertices[0] = origin;

            for (int i = 1; i < segments; ++i, t += increment)
            {
                vertices[i].x = SplineMath.CubicBezier(origin.x, control1.x, control2.x, destination.x, t);
                vertices[i].y = SplineMath.CubicBezier(origin.y, control1.y, control2.y, destination.y, t);
            }

            vertices[segments - 1] = destination;
            DrawPoly(vertices, color);
        }

        void
            make_arcs(float x, float y, float width, float height, float startAngle, float sweepAngle,
                      bool convert_units, bool antialiasing, bool isPieSlice, cpColor color)
        {
            int i;
            float drawn = 0;
            float endAngle;
            bool enough = false;

            endAngle = startAngle + sweepAngle;
            /* if we end before the start then reverse positions (to keep increment positive) */
            if (endAngle < startAngle)
            {
                var temp = endAngle;
                endAngle = startAngle;
                startAngle = temp;
            }

            if (isPieSlice)
            {
                startPoint.x = x + (width / 2);
                startPoint.y = y + (height / 2);
            }

            /* i is the number of sub-arcs drawn, each sub-arc can be at most 90 degrees.*/
            /* there can be no more then 4 subarcs, ie. 90 + 90 + 90 + (something less than 90) */
            for (i = 0; i < 4; i++)
            {
                float current = startAngle + drawn;
                float additional;

                if (enough)
                {
                    if (isPieSlice)
                    {
                        startPoint.x = x + (width / 2);
                        startPoint.y = y + (height / 2);
                        DrawLine(destinationPoint, startPoint, color);
                    }
                    return;
                }

                additional = endAngle - current; /* otherwise, add the remainder */
                if (additional > 90)
                {
                    additional = 90.0f;
                }
                else
                {
                    /* a near zero value will introduce bad artefact in the drawing (#78999) */
                    if ((additional >= -0.0001f) && (additional <= 0.0001f))
                        return;
                    enough = true;
                }

                make_arc((i == 0),    /* only move to the starting pt in the 1st iteration */
                         x, y, width, height,   /* bounding rectangle */
                         current, current + additional, antialiasing, isPieSlice, color);

                drawn += additional;

            }

            if (isPieSlice)
            {
                startPoint.x = x + (width / 2);
                startPoint.y = y + (height / 2);
                DrawLine(destinationPoint, startPoint, color);
            }

        }



        //public override void DrawSolidCircle(cpVect center, float radius, cpVect axis, cpColor color)
        //{

        //    var actualColor = color.ToColor();
        //    var actualCenter = center.ToVector2();


        //    int segments = 16;
        //    double increment = Math.PI * 2.0 / (double)segments;
        //    double theta = 0.0;

        //    Color colorFill = actualColor * 0.5f;

        //    Vector2 v0 = actualCenter + radius * new Vector2((float)Math.Cos(theta), (float)Math.Sin(theta));
        //    theta += increment;

        //    for (int i = 1; i < segments - 1; i++)
        //    {
        //        Vector2 v1 = actualCenter + radius * new Vector2((float)Math.Cos(theta), (float)Math.Sin(theta));
        //        Vector2 v2 = actualCenter + radius * new Vector2((float)Math.Cos(theta + increment), (float)Math.Sin(theta + increment));

        //        _vertsFill[_fillCount * 3].Position = new Vector3(v0, 0.0f);
        //        _vertsFill[_fillCount * 3].Color = colorFill;

        //        _vertsFill[_fillCount * 3 + 1].Position = new Vector3(v1, 0.0f);
        //        _vertsFill[_fillCount * 3 + 1].Color = colorFill;

        //        _vertsFill[_fillCount * 3 + 2].Position = new Vector3(v2, 0.0f);
        //        _vertsFill[_fillCount * 3 + 2].Color = colorFill;

        //        _fillCount++;

        //        theta += increment;
        //    }
        //    DrawCircle(center, radius, color);

        //    DrawSegment(center, (actualCenter.ToCpVect() + axis) * radius, color);
        //}

        public override void DrawSegment(cpVect p1, cpVect p2, cpColor color)
        {
            _vertsLines[_lineCount * 2].Position = p1.ToVector3();// new Vector3(, 0.0f);
            _vertsLines[_lineCount * 2 + 1].Position = p2.ToVector3(); //  //new Vector3(p2, 0.0f);
            _vertsLines[_lineCount * 2].Color = _vertsLines[_lineCount * 2 + 1].Color = color.ToColor();
            _lineCount++;
        }

        public void DrawPoint(cpVect p, float size, cpColor color)
        {
            //var point = p.ToVector2();

            List<cpVect> verts = new List<cpVect>();
            float hs = size / 2.0f;
            verts[0] = p + new cpVect(-hs, -hs);
            verts[1] = p + new cpVect(hs, -hs);
            verts[2] = p + new cpVect(hs, hs);
            verts[3] = p + new cpVect(-hs, hs);

            DrawSolidPolygon(verts, 4, color, true);
        }

        //public void DrawString(int x, int y, string s)
        //{
        //    _stringData.Add(new StringData(x, y, s, null));
        //}


        // AddVertex is called to add another vertex to be rendered. To draw a point,
        // AddVertex must be called once. for lines, twice, and for triangles 3 times.
        // this function can only be called once begin has been called.
        // if there is not enough room in the vertices buffer, Flush is called
        // automatically.
        public void AddVertex(Vector2 vertex, Color color)
        {
            if (!hasBegun)
            {
                throw new InvalidOperationException
                    ("Begin must be called before AddVertex can be called.");
            }

            // are we starting a new primitive? if so, and there will not be enough room
            // for a whole primitive, flush.
            bool newPrimitive = ((positionInBuffer % numVertsPerPrimitive) == 0);

            if (newPrimitive &&
                (positionInBuffer + numVertsPerPrimitive) >= vertices.Length)
            {
                Flush();
            }

            // once we know there's enough room, set the vertex in the buffer,
            // and increase position.
            vertices[positionInBuffer].Position = new Vector3(vertex, 0);
            vertices[positionInBuffer].Color = color;

            positionInBuffer++;
        }

        // End is called once all the primitives have been drawn using AddVertex.
        // it will call Flush to actually submit the draw call to the graphics card, and
        // then tell the basic effect to end.
        public void End()
        {
            if (!hasBegun)
            {
                throw new InvalidOperationException
                    ("Begin must be called before End can be called.");
            }

            // Draw whatever the user wanted us to draw
            Flush();

            hasBegun = false;
        }

        public int _lineCount;
        public int _fillCount;

        // Flush is called to issue the draw call to the graphics card. Once the draw
        // call is made, positionInBuffer is reset, so that AddVertex can start over
        // at the beginning. End will call this to draw the primitives that the user
        // requested, and AddVertex will call this if there is not enough room in the
        // buffer.
        private void Flush()
        {
            if (!hasBegun)
            {
                throw new InvalidOperationException
                    ("Begin must be called before Flush can be called.");
            }

            // no work to do
            if (positionInBuffer == 0)
            {
                return;
            }

            // how many primitives will we draw?
            int primitiveCount = positionInBuffer / numVertsPerPrimitive;

            // submit the draw call to the graphics card
            device.DrawUserPrimitives<VertexPositionColor>(primitiveType, vertices, 0,
                primitiveCount);

            if (_fillCount > 0)
                device.DrawUserPrimitives<VertexPositionColor>(PrimitiveType.TriangleList, _vertsFill, 0, _fillCount);

            if (_lineCount > 0)
                device.DrawUserPrimitives<VertexPositionColor>(PrimitiveType.LineList, _vertsLines, 0, _lineCount);



            // now that we've drawn, it's ok to reset positionInBuffer back to zero,
            // and write over any vertices that may have been set previously.
            positionInBuffer = 0;
        }

        #region Helper functions

        // NumVertsPerPrimitive is a boring helper function that tells how many vertices
        // it will take to draw each kind of primitive.
        static private int NumVertsPerPrimitive(PrimitiveType primitive)
        {
            int numVertsPerPrimitive;
            switch (primitive)
            {
                case PrimitiveType.LineList:
                    numVertsPerPrimitive = 2;
                    break;
                case PrimitiveType.TriangleList:
                    numVertsPerPrimitive = 3;
                    break;
                default:
                    throw new InvalidOperationException("primitive is not valid");
            }
            return numVertsPerPrimitive;
        }

        #endregion

        /// <summary>
        /// draws a polygon given a pointer to CCPoint coordiantes and the number of vertices measured in points.
        /// The polygon can be closed or open and optionally filled with current GL color
        /// </summary>
        public void DrawPoly(cpVect[] vertices, int numOfVertices, bool closePolygon, bool fill, cpColor color)
        {

            if (fill)
            {
                for (int i = 1; i < numOfVertices - 1; i++)
                {
                    AddVertex(new Vector2(vertices[0].x, vertices[0].y), color.ToColor());
                    AddVertex(new Vector2(vertices[i].x, vertices[i].y), color.ToColor());
                    AddVertex(new Vector2(vertices[i + 1].x, vertices[i + 1].y), color.ToColor());
                }
            }
            else
            {
                for (int i = 0; i < numOfVertices - 1; i++)
                {
                    DrawLine(vertices[i], vertices[i + 1], color);
                }

                if (closePolygon)
                {
                    DrawLine(vertices[numOfVertices - 1], vertices[0], color);
                }
            }
        }


        public void DrawLine(cpVect origin, cpVect destination, cpColor color)
        {

            Color clr = color.ToColor();

            var a = origin.ToVector2();
            var b = destination.ToVector2();

            var n = cpVect.Normalize(cpVect.Perp(origin - destination)).ToVector2();

            var lww = LineWidth * 0.5f;
            var nw = n * lww;
            var v0 = b - nw;
            var v1 = b + nw;
            var v2 = a - nw;
            var v3 = a + nw;

            // Triangles from beginning to end
            AddVertex(v1, clr);
            AddVertex(v2, clr);
            AddVertex(v0, clr);
            AddVertex(v1, clr);
            AddVertex(v2, clr);
            AddVertex(v3, clr);

        }
        public const int CircleSegments = 32;
        public override void DrawCircle(cpVect center, float radius, cpColor color)
        {
            //Color col = color.ToColor();

            //Vector2 cen = center.ToVector2();

            const double increment = Math.PI * 2.0 / CircleSegments;
            double theta = 0.0;

            var col = color.ToColor();
            var centr = center.ToVector2();

            for (int i = 0, count = CircleSegments; i < count; i++)
            {
                Vector2 v1 = centr + radius * new Vector2((float)Math.Cos(theta), (float)Math.Sin(theta));
                Vector2 v2 = centr +
                             radius *
                             new Vector2((float)Math.Cos(theta + increment), (float)Math.Sin(theta + increment));

                AddVertex(v1, col);
                AddVertex(v2, col);

                theta += increment;
            }
        }




        public override void DrawSegment(cpVect from, cpVect to, float radius, cpColor color)
        {
            var colorFill = color.ToColor();


            var a = from.ToVector2();
            var b = to.ToVector2();

            var n = cpVect.Perp(from - to).Normalize().ToVector2();

            var nw = n * radius;
            var v0 = b - nw;
            var v1 = b + nw;
            var v2 = a - nw;
            var v3 = a + nw;

            // Triangles from beginning to end
            AddVertex(v1, colorFill);
            AddVertex(v2, colorFill);
            AddVertex(v0, colorFill);

            AddVertex(v1, colorFill);
            AddVertex(v2, colorFill);
            AddVertex(v3, colorFill);
        }

        public override void DrawSolidCircle(cpVect center, float radius, cpVect axis, cpColor color)
        {
            //CCColor4B fillColor = color;

            const double increment = Math.PI * 2.0 / CircleSegments;
            double theta = 0.0;

            var colorFill = color.ToCCColor4B() * 0.5f;
            var centr = center.ToVector2();

            Vector2 v0 = center.ToVector2() + radius * new Vector2((float)Math.Cos(theta), (float)Math.Sin(theta));
            theta += increment;

            for (int i = 1; i < CircleSegments - 1; i++)
            {
                var v1 = centr + radius * new Vector2((float)Math.Cos(theta), (float)Math.Sin(theta));
                var v2 = centr +
                         radius * new Vector2((float)Math.Cos(theta + increment), (float)Math.Sin(theta + increment));

                AddVertex(v0, colorFill);
                AddVertex(v1, colorFill);
                AddVertex(v2, colorFill);

                theta += increment;
            }
            DrawCircle(center, radius, color);

            DrawSegment(center, center + axis * radius, color);
        }

        public override void DrawString(int x, int y, string format, params object[] objects)
        {
            //throw new NotImplementedException();
        }
    }

    internal static class SplineMath
    {
        // CatmullRom Spline formula:
        /// <summary>
        /// See http://en.wikipedia.org/wiki/Cubic_Hermite_spline#Cardinal_spline
        /// </summary>
        /// <param name="p0">Control point 1</param>
        /// <param name="p1">Control point 2</param>
        /// <param name="p2">Control point 3</param>
        /// <param name="p3">Control point 4</param>
        /// <param name="tension"> The parameter c is a tension parameter that must be in the interval (0,1). In some sense, this can be interpreted as the "length" of the tangent. c=1 will yield all zero tangents, and c=0 yields a Catmull–Rom spline.</param>
        /// <param name="t">Time along the spline</param>
        /// <returns>The point along the spline for the given time (t)</returns>
        internal static cpVect CCCardinalSplineAt(cpVect p0, cpVect p1, cpVect p2, cpVect p3, float tension, float t)
        {
            if (tension < 0f)
            {
                tension = 0f;
            }
            if (tension > 1f)
            {
                tension = 1f;
            }
            float t2 = t * t;
            float t3 = t2 * t;

            /*
             * Formula: s(-ttt + 2tt - t)P1 + s(-ttt + tt)P2 + (2ttt - 3tt + 1)P2 + s(ttt - 2tt + t)P3 + (-2ttt + 3tt)P3 + s(ttt - tt)P4
             */
            float s = (1 - tension) / 2;

            float b1 = s * ((-t3 + (2 * t2)) - t); // s(-t3 + 2 t2 - t)P1
            float b2 = s * (-t3 + t2) + (2 * t3 - 3 * t2 + 1); // s(-t3 + t2)P2 + (2 t3 - 3 t2 + 1)P2
            float b3 = s * (t3 - 2 * t2 + t) + (-2 * t3 + 3 * t2); // s(t3 - 2 t2 + t)P3 + (-2 t3 + 3 t2)P3
            float b4 = s * (t3 - t2); // s(t3 - t2)P4

            float x = (p0.x * b1 + p1.x * b2 + p2.x * b3 + p3.x * b4);
            float y = (p0.y * b1 + p1.y * b2 + p2.y * b3 + p3.y * b4);

            return new cpVect(x, y);
        }

        // Bezier cubic formula:
        //	((1 - t) + t)3 = 1 
        // Expands to 
        //   (1 - t)3 + 3t(1-t)2 + 3t2(1 - t) + t3 = 1 
        internal static float CubicBezier(float a, float b, float c, float d, float t)
        {
            float t1 = 1f - t;
            return ((t1 * t1 * t1) * a + 3f * t * (t1 * t1) * b + 3f * (t * t) * (t1) * c + (t * t * t) * d);
        }

        internal static float QuadBezier(float a, float b, float c, float t)
        {
            float t1 = 1f - t;
            return (t1 * t1) * a + 2.0f * (t1) * t * b + (t * t) * c;

        }
    }

}
