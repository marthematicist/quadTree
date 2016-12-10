function setupGlobalVariables() {
	// CANVAS VARIABLES
	{
		// set canvas size to fill the window
		xRes = windowWidth;
		yRes = windowHeight;
		winArea = xRes * yRes;
		minRes = min( xRes , yRes );
		maxRes = max( xRes , yRes );
	}
	
	// DRAW VARIABLES
	{
		// general draw variables
		bgColor = color( 0 , 0 , 0 , 2 );
		// tree draw variables
		drawTreeFill = true;
		drawTreeDiv = true;
		divColor = color( 128 , 128 , 128 , 128 );
		treeFillColor = color( 128 , 128 , 128 , 64 );
		divWeight = 0.5
		// body draw variables
		drawBodies = true;
		bodyDiam = 2;
		bodyAlpha = 30;
		bodyColor = color( 255 , 255 , 255 , bodyAlpha );
		fillAlpha = 3;
		baseFillColor = color( 0 , 200 , 255 , fillAlpha );
		minLerpAmt = 0.0;
		maxLerpAmt = 0.8;
		randomColor = true;
	}
	
	// SIMULATION VARIABLES
	{
		// number of bodies
		numBodies = 80;
		// simulation area
		simArea = 100;
		// linear conversion factor: sim to window
		sim2WinFactor = sqrt( winArea / simArea );
		// linear conversion factor: window to sim
		win2SimFactor = sqrt( simArea / winArea );
		// dimensions of the simulation
		xExt = xRes * win2SimFactor;
		yExt = yRes * win2SimFactor;
		// bounds of simulation
		xMin = -0.5*xExt;
		yMin = -0.5*yExt;
		xMax = 0.5*xExt;
		yMax = 0.5*yExt;
		// simulation center (vector)
		simCenter = createVector( 0.5*( xMin + xMax ) , 0.5*( yMin + yMax ) );
		// body mass variables
		avgMass = 1;
		minMass = 0.1;
		maxMass = 5;
		// probability of negative particle
		negProb = 0.5;
		// PHYSICS CONSTANTS
		reversePhysics = false;
		dt = 1.0 / ( 200 );
		edgeSpringConstant = 1000;
		frictionConstant = 0.1;
		universalConstant = 1;
		epsilon = min(xExt,yExt)*0.1;
		// max recursion depth
		maxDepth = 15;
		maxRecDepth = 0;
	}
}

// functions to convert linear distance and vectors from sim to window
function sim2Win( a ) {
	return a*sim2WinFactor;
};
function sim2WinVect( a ) {
	return createVector( (a.x - xMin)*sim2WinFactor , 
						 (a.y - yMin)*sim2WinFactor );
};

// CLASS Body
var Body = function() {
	// x = position (randomized)
	this.x = createVector( random( xMin , xMax ) , random( yMin , yMax ) );
	// v = velocity
	this.v = createVector( 0 , 0 );
	// a = acceleration
	this.a = createVector( 0 , 0 );
	// m = mass
	this.m = random( minMass , maxMass );
	// c = color
	this.c = bodyColor;
	// p = charge (1 or -1)
	if( random(0,1) < negProb ) { this.p = -1; } else { this.p = 1; }
	// treeLoc = location in QuadTree: array of integers
	this.treeLoc = [];
	if( randomColor ) {
		var rc = color( random(0,255) , random(0,255) , random(0,255) , fillAlpha );
		var la = random( minLerpAmt , maxLerpAmt );
		this.c = lerpColor( baseFillColor , rc , la );
	};
	
	// method to draw the body to the screen
	this.draw = function() {
		if( this.m > 0 ) {
			//fill( 255-red(this.c) , 255-blue(this.c) , 255-green(this.c) , bodyAlpha );
			fill( 255 , 255 , 255 , bodyAlpha );
			var c = sim2WinVect( this.x );
			ellipse( c.x , c.y , bodyDiam , bodyDiam );
		}
	}
	
	
};

// function to create a new QuadTree object
function createQuadTree( center , halfDimX , halfDimY ) {
	var qt = new QuadTree( center , halfDimX , halfDimY );
	qt.isRoot = true;
	return qt;
}

// CLASS QuadTree
var QuadTree = function( center , halfDimX , halfDimY ) {
	this.isRoot = false;
	this.hasChildren = false;
	this.hasBody = false;
	this.center = center;
	this.halfDimX = halfDimX;
	this.halfDimY = halfDimY;
	this.children = new Array(4);
	this.parent = 0;
	this.body = 0;
	this.numGenerations = 0;
	this.depth = 0;
	
	// method to determine which child a body belongs in
	// returns integer: NE = 0 , NW = 1 , SW = 2 , SE = 3
	this.whichChild = function( b ) {
		// if north
		if( b.x.y > this.center.y ) {
			// if east (NE)
			if( b.x.x > this.center.x ) { return 0;	}
			// otherwise west (NW)
			else { return 1; }
		}
		// otherwise south
		else {
			// if east (SE)
			if( b.x.x > this.center.x ) { return 3; }
			// otherwise west (SW)
			else { return 2; }
		}
	}
	
	// method to add children to a tree
	this.createChildren = function() {
		this.children = new Array(4);
		var c0 = createVector( this.center.x + 0.5*this.halfDimX ,
							   this.center.y + 0.5*this.halfDimY );
		this.children[0] = new QuadTree( c0 , 0.5*halfDimX , 0.5*halfDimY );
		this.children[0].parent = this;
		this.children[0].depth = this.depth + 1;
		var c1 = createVector( this.center.x - 0.5*this.halfDimX ,
							   this.center.y + 0.5*this.halfDimY );
		this.children[1] = new QuadTree( c1 , 0.5*halfDimX , 0.5*halfDimY );
		this.children[1].parent = this;
		this.children[1].depth = this.depth + 1;
		var c2 = createVector( this.center.x - 0.5*this.halfDimX ,
							   this.center.y - 0.5*this.halfDimY );
		this.children[2] = new QuadTree( c2 , 0.5*halfDimX , 0.5*halfDimY );
		this.children[2].parent = this;
		this.children[2].depth = this.depth + 1;
		var c3 = createVector( this.center.x + 0.5*this.halfDimX ,
							   this.center.y - 0.5*this.halfDimY );
		this.children[3] = new QuadTree( c3 , 0.5*halfDimX , 0.5*halfDimY );
		this.children[3].parent = this;
		this.children[3].depth = this.depth + 1;
		this.hasChildren = true;
		
		this.changeParentNumGenerations( 1 );
		
		if( this.depth > maxRecDepth ) { maxRecDepth = this.depth; }
	}
	
	// method to increment parent generation counts
	this.changeParentNumGenerations = function( a ) {
		//console.log( this.numGenerations );
		this.numGenerations += a;
		if( !this.isRoot ) {
			this.parent.changeParentNumGenerations( a );
		}
	};
	
	// method to add a body to a tree (recursive)
	this.addBody = function( b ) {
		// if there are already children, add the body to the
		// appropriate child
		if( this.hasChildren ) {
			this.children[ this.whichChild( b ) ].addBody( b );
		}
		// otherwise there are no children
		else {
			// if there is already a body in this tree,
			if( this.hasBody ) {
				//  and if we haven't reached max depth
				if( this.depth <= maxDepth ) {
					// create children and add the existing and new
					// bodies to the appropriate children
					this.createChildren();
					this.children[ this.whichChild( this.body ) ].addBody( this.body );
					this.children[ this.whichChild( b ) ].addBody( b );
					this.hasBody = false;
				} 
				// if at max depth
				else {
					// combine the bodies by setting the mass of one to 0
					// and keeping the other with mass = sum of the bodies
					// momentum is conserved
					// charge will be the charge of the more massive body
					var a = new Body();
					// new position
					a.x = p5.Vector.lerp( b.x , this.body.x , b.m / ( b.m + this.body.m ) );
					// momentum for bodies
					var m1 = p5.Vector.mult( b.v , b.m );
					var m2 = p5.Vector.mult( this.body.v , this.body.m );
					// new velocity (momentum conserved)
					a.v = p5.Vector.mult( p5.Vector.add( m1 , m2 ) , 1 / ( b.m + this.body.m ) );
					// new mass
					a.m = b.m + this.body.m;
					// new color
					a.c = lerpColor( b.c , this.body.c , b.m / ( b.m + this.body.m ) );
					// new charge
					if( b.m > this.body.m ) { a.p = b.p; } 
					else { a.p = this.body.p; }
					// set this.body to new body
					this.body.x = createVector( a.x.x , a.x.y );
					this.body.v = createVector( a.v.x , a.v.y );
					this.body.a = createVector( 0 , 0 );
					this.body.c = a.c;
					this.body.m = a.m;
					this.body.p = a.p;
					// set other body for removal
					b.m = 0;
				}
			}
			// otherwise there is no body, so place the body in this tree
			else {
				this.body = b;
				this.hasBody = true;
			}
		}
	}
	
	// method to draw divisions
	this.drawDiv = function() {
		if( this.hasChildren ) {
			var c = sim2WinVect( this.center );
			var hdx = sim2Win( this.halfDimX );
			var hdy = sim2Win( this.halfDimY );
			line( c.x - hdx , c.y , c.x + hdx , c.y );
			line( c.x , c.y - hdy , c.x , c.y + hdy );
			this.children[0].drawDiv();
			this.children[1].drawDiv();
			this.children[2].drawDiv();
			this.children[3].drawDiv();
		}
	}
	
	// method to color children with bodies
	this.fillChildren = function() {
		if( this.hasBody ) {
			fill( this.body.c );
			var c = sim2WinVect( this.center );
			var hdx = sim2Win( this.halfDimX );
			var hdy = sim2Win( this.halfDimY );
			rect( c.x - hdx , c.y - hdy , 2*hdx , 2*hdy );
		}
		if( this.hasChildren ) {
			this.children[0].fillChildren();
			this.children[1].fillChildren();
			this.children[2].fillChildren();
			this.children[3].fillChildren();
		}
	}
};

// CLASS BodySim
var BodySim = function( num ) {
	// number of bodies
	this.N = num;
	// array of bodies
	this.B = new Array( num );
	// quadtree
	this.T = createQuadTree( simCenter , 0.5*xExt , 0.5*yExt );
	// define bodies and populate the quadTree
	for( var i = 0 ; i < numBodies ; i++ ) {
		this.B[i] = new Body();
		this.T.addBody( this.B[i] );
	}
	
	
	// method to draw the bodies
	this.drawBodies = function() {
		noStroke();
		for( var i = 0 ; i < this.N ; i++ ) {
			this.B[i].draw();
		}
	};
	
	// method to draw the QuadTree divisions and color the populated children
	this.drawTree = function( fillOn , divOn ) {
		if( fillOn ) {
			// fill the tree
			noStroke();
			this.T.fillChildren();
		}
		if( divOn ) {
			// draw the divisions
			stroke( divColor );
			strokeWeight( divWeight );
			this.T.drawDiv();
		}
	};
	
	// method to update the QuadTree
	this.updateTree = function() {
		this.T = createQuadTree( simCenter , 0.5*xExt , 0.5*yExt );
		// populate the quadTree
		for( var i = 0 ; i < this.N ; i++ ) {
			this.T.addBody( this.B[i] );
		}
	};
	
	// method to zero all accelerations
	this.zeroAccelerations = function() {
		for( var i = 0 ; i < this.N ; i++ ) {
			this.B[i].a = createVector( 0 , 0 );
		}
	};
	
	// method to apply friction forces to all bodies
	this.applyFrictionForces = function() {
		for( var i = 0 ; i < this.N ; i++ ) {
			dA = createVector( this.B[i].v.x , this.B[i].v.y );
			dA.mult( -frictionConstant*dA.mag() );
			this.B[i].a.add( dA );
		}
	};
	 
	// method to apply mutual forces to all bodies (BRUTE)
	this.applyMutualForcesBrute = function() {
		// do for each body
		for( var i = 0 ; i < this.N - 1 ; i++ ) {
			// apply resultant forces from all other bodies
			for( var j = i + 1 ; j < this.N ; j++ ) {
				// get the distance between the bodies
				var d = p5.Vector.dist( this.B[i].x , this.B[j].x );
				// calculate the force magnitude (neglecting mass)
				var f = universalConstant / pow( d * d + epsilon * epsilon , 1.5 );
				// calculate and normalize a direction vector from body j to body i
				var dAi = p5.Vector.sub( this.B[i].x , this.B[j].x );
				dAi.normalize();
				// make a copy
				var dAj = createVector( dAi.x , dAi.y );
				// get the masses of the bodies
				var mj = this.B[j].m;
				var mi = this.B[i].m;
				// find the final change in acceleration
				var rev = 1;
				if( reversePhysics ) { rev = -1; }
				if( this.B[i].p * this.B[j].p * rev < 0 ) {
					// if charges are different, repel
					dAi.mult( f * abs(mj) );
					dAj.mult( -f * abs(mi) );
				} else {
					// if charges are same, attract
					dAi.mult( -f * abs(mj) );
					dAj.mult( f * abs(mi) );
				}
				// update acceleration vectors
				this.B[i].a.add( dAi );
				this.B[j].a.add( dAj );
			}
		}
	};
	
	// method to apply edge forces
	this.applyEdgeForces = function() {
		for( var i = 0 ; i < this.N ; i++ ) {
			var x = this.B[i].x.x;
			var y = this.B[i].x.y;

			if( this.B[i].x.x < xMin ) {
				this.B[i].x.x = xMin;
				this.B[i].v.x = abs( this.B[i].v.x );
			}
			if( this.B[i].x.y < yMin ) {
				this.B[i].x.y = yMin;
				this.B[i].v.y = abs( this.B[i].v.y );
			}
			if( this.B[i].x.x > xMax ) {
				this.B[i].x.x = xMax;
				this.B[i].v.x = -abs( this.B[i].v.x );
			}
			if( this.B[i].x.y > yMax ) {
				this.B[i].x.y = yMax;
				this.B[i].v.y = -abs( this.B[i].v.y );
			}
			/*
			var m = abs(this.B[i].m);
			var f;
			var dA;
			if( x < xMin ) {
				f = edgeSpringConstant * ( xMin - x );
				dA = createVector( f / m , 0 );
				this.B[i].a.add( dA );
			}
			if( y < yMin ) {
				f = edgeSpringConstant * ( yMin - y );
				dA = createVector( 0, f / m );
				this.B[i].a.add( dA );
			}
			if( x > xMax ) {
				f = edgeSpringConstant * ( x - ( xMax ) );
				dA = createVector( -f / m , 0 );
				this.B[i].a.add( dA );
			}
			if( y > yMax ) {
				f = edgeSpringConstant * ( y - ( yMax ) );
				dA = createVector( 0 , -f / m );
				this.B[i].a.add( dA );
			}
			* */
		}
	};
	
	// method to evolve the simulation 1/2 step 
	this.evolveHalfStep = function() {
		this.zeroAccelerations();
		this.applyMutualForcesBrute();
		this.applyEdgeForces();
		this.applyFrictionForces();
		for( var i = 0 ; i < this.N ; i++ ) {
			this.B[i].v.add( p5.Vector.mult( this.B[i].a , dt / 2 ) );
		}
	};
	
	// method to evolve the simulation num full steps
	this.evolveFullStep = function( num ) {
		for( var n = 0 ; n < num ; n++ ) {
			for( var i = 0 ; i < this.N ; i++ ) {
				this.B[i].x.add( p5.Vector.mult( this.B[i].v , dt ) );
			}
			this.zeroAccelerations();
			this.applyMutualForcesBrute();
			this.applyEdgeForces();
			this.applyFrictionForces();
			for( var i = 0 ; i < this.N ; i++ ) {
				this.B[i].v.add( p5.Vector.mult( this.B[i].a , dt ) );
			}
		}
	};
	
	// method to remove bodies with zero mass
	this.removeZeroMasses = function() {
		ind = [];
		for( var i = 0 ; i < this.N ; i++ ) {
			if( this.B[i].m === 0 ) {
				append( ind,  i );
				console.log( "removed a body!" );
			}
		}
		this.N -= ind.length;
		reverse( ind );
		for( var i = 0 ; i < ind.length ; i++ ) {
			this.B.splice( ind[i] , 1 );
		}
	}
	
};


function setup() {
	// set up global variables
	setupGlobalVariables();
	// define canvas
	createCanvas( xRes , yRes );
	
	// initialize simulation
	S = new BodySim( numBodies );
	// evolve the simulation 1/2 step
	S.evolveHalfStep();
	maxGen = 0;
	maxRecDepth = 0;
	
	background( 0 , 0 , 0 );
}

function draw() {
	// draw background
	background( bgColor );
	
	// set one body under mouse
	if( false ) {
		S.B[0].v = createVector( 0 , 0 );
		S.B[0].a = createVector( 0 , 0 );
		S.B[0].m = 50*maxMass;
		S.B[0].x = createVector( xMin + win2SimFactor*mouseX , 
								 yMin + win2SimFactor*mouseY );
		S.B[0].p = -1;
	}
	
	/*
	if( mouseIsPressed ) {
		reversePhysics = true;
	} else {
		reversePhysics = false;
	}
	* */
	
	// evolve the simulation full steps
	S.evolveFullStep(1);
	S.updateTree();
	S.removeZeroMasses();
	// draw QuadTree
	S.drawTree( drawTreeFill , drawTreeDiv );
	
	// draw bodies
	if( drawBodies ) { S.drawBodies(); }
	if( S.T.numGenerations > maxGen ) {
		maxGen = S.T.numGenerations;
	}
	console.log( maxGen , maxRecDepth );
}

function mouseClicked() {
	if( reversePhysics ) {
		reversePhysics = false; 
	} else { 
		reversePhysics = true; 
	}
}



