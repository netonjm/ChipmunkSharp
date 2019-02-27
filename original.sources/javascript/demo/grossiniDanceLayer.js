/* Copyright (c) 2007 Scott Lembcke
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
 
var Dance = function() {

	Demo.call(this);

	var space = this.space;
	
	space.gravity = v(0, -300);
	
	this.addFloor();
	this.addWalls();
		
	var width = 50;
	var height = 60;
	var mass = width * height * 1/1000;
	var rock = space.addBody(new cp.Body(mass, cp.momentForBox(mass, width, height)));
	rock.setPos(v(500, 100));
	rock.setAngle(1);
	shape = space.addShape(new cp.BoxShape(rock, width, height));
	shape.setFriction(0.3);
	shape.setElasticity(0.3);
		
	};

	Dance.prototype = Object.create(Demo.prototype);
	
	Dance.prototype.update = function(dt)
{
	var steps = 3;
	dt /= steps;
	for (var i = 0; i < 3; i++){
		this.space.step(dt);
	}
}


addDemo('Dance', Dance);

