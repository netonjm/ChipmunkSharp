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

using ChipmunkSharp.Constraints;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace ChipmunkSharp
{

    public class cpContactBufferHeader
    {
        public int stamp;
        public cpContactBufferHeader next;
        public int numContacts;
    };


    public partial class cpSpace
    {


        // **** Post Step Callback Functions
        public void AddPostStepCallback(Action func)
        {
            cpEnvironment.assertSoft(this.isLocked,
         "Adding a post-step callback when the space is not locked is unnecessary. " +
         "Post-step callbacks will not called until the end of the next call to cpSpaceStep() or the next query.");

            this.postStepCallbacks.Add(func);
        }

        public void runPostStepCallbacks()
        {
            // Don't cache length because post step callbacks may add more post step callbacks
            // directly or indirectly.
            for (var i = 0; i < this.postStepCallbacks.Count; i++)
                this.postStepCallbacks[i]();
            this.postStepCallbacks.Clear();

        }

        //MARK: Locking Functions

        public void Lock()
        {
            locked++;
        }

        public void Unlock(bool runPostStep)
        {

            this.locked--;
            cpEnvironment.assertSoft(this.locked >= 0, "Internal Error: Space lock underflow.");

            if (this.locked == 0 && runPostStep)
            {
                var waking = this.rousedBodies;
                for (var i = 0; i < waking.Count; i++)
                    this.activateBody(waking[i]);

                this.runPostStepCallbacks();
            }

        }





























        static bool QueryReject(cpShape a, cpShape b)
        {
            return (
                // BBoxes must overlap
                !a.bb.Intersects(b.bb)
                // Don't collide shapes attached to the same body.
                || a.body == b.body
                // Don't collide objects in the same non-zero group
                || (a.group != 0 && a.group == b.group)
                // Don't collide objects that don't share at least on layer.
                || !(a.layers != 0 & b.layers != 0)
                // Don't collide infinite mass objects
                || (a.body.Mass == cpEnvironment.Infinity && b.body.Mass == cpEnvironment.Infinity)
            );
        }

        // Callback from the spatial hash.
        public List<ContactPoint> collideShapes(cpShape a, cpShape b)
        {
            cpEnvironment.assertWarn((a as ICollisionShape).collisionCode <= (b as ICollisionShape).collisionCode, "Collided shapes must be sorted by type");
            return (a as ICollisionShape).collisionTable[(b as ICollisionShape).collisionCode](a, b);
            //return null;

        }

        private void UpdateFunc(cpShape shape)
        {
            var body = shape.body;
            shape.update(body.Position, body.Rotation);
        }


    }
}


//#define CP_CONTACTS_BUFFER_SIZE ((CP_BUFFER_BYTES - sizeof(cpContactBufferHeader))/sizeof(cpContact))
//struct cpContactBuffer {
//    cpContactBufferHeader header;
//    cpContact contacts[CP_CONTACTS_BUFFER_SIZE];
//} cpContactBuffer;

//public static cpContactBufferHeader  cpSpaceAllocContactBuffer(cpSpace space)
//{
//    cpContactBuffer buffer = (cpContactBuffer)cpcalloc(1, sizeof(cpContactBuffer));
//    cpArrayPush(space.allocatedBuffers, buffer);
//    return (cpContactBufferHeader *)buffer;
//}

//static cpContactBufferHeader cpContactBufferHeaderInit(cpContactBufferHeader header, int stamp, cpContactBufferHeader splice)
//{
//    header.stamp = stamp;
//    header.next = (splice != null ? splice.next : header);
//    header.numContacts = 0;
//    return header;
//}

//void cpSpacePushFreshContactBuffer(cpSpace space)
//{
//    int stamp = space.stamp;

//    //List<cpContactBufferHeader> head = ;

//    if(space.contactBuffersHead!=null){
//        // No buffers have been allocated, make one
//        space.contactBuffersHead = new List<cpContact>();
//    } else if(stamp - head.next.stamp > space.collisionPersistence){
//        // The tail buffer is available, rotate the ring
//    cpContactBufferHeader tail = head.next;
//        space.contactBuffersHead = cpContactBufferHeaderInit(tail, stamp, tail);
//    } else {
//        // Allocate a new buffer and push it into the ring
//        cpContactBufferHeader *buffer = cpContactBufferHeaderInit(cpSpaceAllocContactBuffer(space), stamp, head);
//        space.contactBuffersHead = head.next = buffer;
//    }
//}


//cpContact cpContactBufferGetArray(cpSpace space)
//{
//    if (space.contactBuffersHead.numContacts + CP_MAX_CONTACTS_PER_ARBITER > CP_CONTACTS_BUFFER_SIZE)
//    {
//        // contact buffer could overflow on the next collision, push a fresh one.
//        cpSpacePushFreshContactBuffer(space);
//    }

//    cpContactBufferHeader head = space.contactBuffersHead;
//    return ((cpContactBuffer)head).contacts + head.numContacts;
//}

//void
//cpSpacePushContacts(cpSpace space, int count)
//{
//    cpAssertHard(count <= CP_MAX_CONTACTS_PER_ARBITER, "Internal Error: Contact buffer overflow!");
//    space.contactBuffersHead.numContacts += count;
//}

//static void cpSpacePopContacts(cpSpace space, int count)
//{
//    space.contactBuffersHead.numContacts -= count;
//}

//MARK: Collision Detection Functions

//static void cpSpaceArbiterSetTrans(cpShape shapes, cpSpace space)
//{
//    if (space.pooledArbiters.Count == 0)
//    {
//        // arbiter pool is exhausted, make more
//        int count = CP_BUFFER_BYTES / sizeof(cpArbiter);
//        cpAssertHard(count, "Internal Error: Buffer size too small.");

//        cpArbiterbuffer = (cpArbiter)cpcalloc(1, CP_BUFFER_BYTES);
//        cpArrayPush(space.allocatedBuffers, buffer);

//        for (int i = 0; i < count; i++) cpArrayPush(space.pooledArbiters, buffer + i);
//    }

//    return cpArbiterInit(space.pooledArbiters, shapes[0], shapes[1]);
//}
