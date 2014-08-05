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

	public class cpContact
	{

		//public cpVect p, n;
		//public float dist;

		public cpVect r1, r2;
		public float nMass, tMass, bounce;

		public float jnAcc, jtAcc, jBias;
		public float bias;

		public ulong hash;

		public override string ToString()
		{
			return string.Format("{0}: p({1}),n({2})", hash, r1, r2);
		}

		public cpContact(cpVect r1, cpVect r2, ulong hash)
		{
			Init(r1, r2, hash);
		}

		//public cpContact Clone()
		//{
		//	cpContact tmp = new cpContact(p, n, dist, hash);
		//	return tmp;
		//}

		public void Init(cpVect r1, cpVect r2, ulong hash)
		{
			this.r1 = r1;
			this.r2 = r2;
			//this.dist = dist;

			//	this.r1 = this.r2 = cpVect.Zero;
			this.nMass = this.tMass = this.bounce = this.bias = 0;

			this.jnAcc = this.jtAcc = this.jBias = 0;
			this.hash = hash;
			cp.numContacts++;
		}

		//public void Draw(cpDebugDraw m_debugDraw)
		//{
		//	m_debugDraw.DrawPoint(r1, 1, cpColor.Red);
		//	m_debugDraw.DrawPoint(r2, 1, cpColor.Red);
		//}

	};

}
