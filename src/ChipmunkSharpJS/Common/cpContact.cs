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

        public cpVect p, n;
        public float dist;

        public cpVect r1, r2;
        public float nMass, tMass, bounce;

        public float jnAcc, jtAcc, jBias;
        public float bias;

        public int hash;

        public cpContact(cpVect p, cpVect n, float dist, int hash)
        {
            Init(p, n, dist, hash);
        }

        public cpContact Clone()
        {
            cpContact tmp = new cpContact(p, n, dist, hash);
            return tmp;
        }

        public void Init(cpVect p, cpVect n, float dist, int hash)
        {
            this.p = p;
            this.n = n;
            this.dist = dist;

            this.r1 = this.r2 = cpVect.ZERO;
            this.nMass = this.tMass = this.bounce = this.bias = 0;

            this.jnAcc = this.jtAcc = this.jBias = 0;
            this.hash = hash;
            cpEnvironment.numContacts++;
        }
    };

}
