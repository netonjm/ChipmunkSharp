![alt tag](http://files.slembcke.net/chipmunk/logo/logo1_med.png)

h2. NOTE!

The master branch is the in progress Chipmunk2D 7.0. The documentation on Chipmunk2D's website may not completely match. While the code should be pretty stable (there are some unit tests), the API is still evolving. You can check out the 6.x branch if you want the last released version.

h2. ABOUT:

Chipmunk2D is a simple, lightweight, fast and portable 2D rigid body physics library written in C#. It's licensed under the unrestrictive, OSI approved MIT license.

h2. FEATURES:

* Designed specifically for 2D video games.
* Circle, convex polygon, and beveled line segment collision primitives.
* Multiple collision primitives can be attached to a single rigid body.
* Fast broad phase collision detection by using a bounding box tree with great temporal coherence or a spatial hash.
* Extremely fast impulse solving by utilizing Erin Catto's contact persistence algorithm.
* Supports sleeping objects that have come to rest to reduce the CPU load.
* Support for collision event callbacks based on user definable object types types.
* Flexible collision filtering system with layers, exclusion groups and callbacks.
** Can be used to create all sorts of effects like one way platforms or buoyancy areas. (Examples included)
* Supports nearest point, segment (raycasting), shape and bounding box queries to the collision detection system.
* Collision impulses amounts can be retrieved for gameplay effects, sound effects, etc.
* Large variety of joints - easily make vehicles, ragdolls, and more.
* Joint callbacks.
** Can be used to easily implement breakable or animated joints. (Examples included)
* Maintains a contact graph of all colliding objects.
* Lightweight C99 implementation with no external dependencies outside of the Std. C library.
* "Many language bindings available":http://chipmunk2d.net/bindingsAndPorts.php.
* Simple, read the "documentation":http://chipmunk2d.net/documentation.php and see!
* Unrestrictive MIT license


BUILDING:

TODO


GET UP TO DATE:

If you got the source from a point release download, you might want to consider getting the latest source from GitHub. Bugs are fixed and new features are added regularly. Big changes are done in branches and tested before merging them in it's rare for the point release downloads to be better or more bug free than the latest code.


GETTING STARTED:

First of all, you can find the C API documentation in the doc/ directory.

A good starting point is to take a look at the included Demo application. The demos all just set up a Chipmunk simulation space and the demo app draws the graphics directly out of that. This makes it easy to see how the Chipmunk API works without worrying about the graphics code. You are free to use the demo drawing routines in your own projects, though it is certainly not the recommended way of drawing Chipmunk objects as it pokes around at the undocumented/private APIs of Chipmunk.


h2. SUPPORT:

The best way to get support is to visit the "Chipmunk Forums https://chipmunksharp.codeplex.com/discussions
