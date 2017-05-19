function setupGlobalVariables() {
	
	// version number
	versionNumber = '0.65';
	// CANVAS VARIABLES
	{
		// set canvas size to fill the window
		xRes = windowWidth;
		yRes = windowHeight;
		// area of window
		winArea = xRes * yRes;
		// min and max resolution
		minRes = min( xRes , yRes );
		maxRes = max( xRes , yRes );
	}

	// TIMING VARIABLES
	{
	  // used to track framerate
		frameTimer = millis();
		// time of last click (for double-click detection)
		clickTimer = millis();
		// min time between clicks for double-click detection (ms)
		doubleClickTime = 400;
		// time that setup was completed
		startTimer = 0;
		// amount of time to wait after setup before beginning simulation (title screen time)
		startWaitTime = 4000;
		// used to clear the screen after title
		clearFirstTime = true;
		// time that mode was last changed
		modeChangeTimer = millis();
		// amt of time to display the mode after a change
		modeChangeDisplayTime = 4000;
		
	}
	
	// SIMULATION VARIABLES
	{
		// number of bodies
		numBodies = 64;
		// initial conditions
		initCond = 1;
		// simulation area
		simArea = 100;
		// linear conversion factor: sim to window
		sim2WinFactor = sqrt( winArea / simArea );
		// linear conversion factor: window to sim
		win2SimFactor = sqrt( simArea / winArea );
		// dimensions of the simulation
		xExt = xRes * win2SimFactor;
		yExt = yRes * win2SimFactor;
		// the minimum dimension of the simulation
		minExt = min( xExt , yExt );
		// bounds of simulation
		xMin = -0.5*xExt;
		yMin = -0.5*yExt;
		xMax = 0.5*xExt;
		yMax = 0.5*yExt;
		// simulation center (vector)
		simCenter = createVector( 0.5*( xMin + xMax ) , 0.5*( yMin + yMax ) );
		// body mass variables
		totalMass = 400;
		massDev = 0.95;
		avgMass = totalMass/numBodies;
		minMass = (1-massDev)*avgMass;
		maxMass = (1+massDev)*avgMass;
		// PHYSICS CONSTANTS
		reversePhysics = false;
		// simulation time per frame
		dt = 1.0 / ( 400 );
		// Edge velocity dampening
		edgeNormalDamping = 0.05;
		// Friction coefficients
		frictionConstantAttract = 0.001;
		frictionConstantRepel = 0.2;
		// Gravity constant
		universalConstant = 1;
		// gravity smoothing factor (0.4 optimum)
		epsilon = 0.4;
		// ratio for Barnes-Hut tree method (0.3 optimum)
		theta = 0.3;
		// mutual force comuptation algorithm: Brute or tree (Barnes-Hut)
		bruteMethod = false;
		// max recursion depth
		maxDepth = 14;
		
	}
	
	// DRAW VARIABLES
	{
		// artMode - bodies not displayed
		artMode = false;
		// general draw variables
		bgColor = color( 0 , 0 , 0 , 10  );
		// tree draw variables
		drawTreeFill = true;
		drawTreeDiv = true;
		drawCOM = false;
		// threshold for deciding whther to draw center-of-mass
		comDrawThreshold = 3.5*avgMass;
		comSizeFactor = 1;
		// color of tree divisions
		divColor = color( 192 , 192 , 192 , 64 );
		// thickness of divisions
		divWeight = 1;
		// center of mass color
		comColor = color( 196 , 196 , 0 , 1 );
		// body draw variables
		if( artMode ) {	drawBodies = false; }
		else { drawBodies = true; }
		// body diameter
		bodyDiam = minRes*0.0055;
		// body transparency
		bodyAlpha = 128;
		// body color
		bodyColor = color( 255 , 255 , 255 , bodyAlpha );
		// tree fill transparency
		fillAlpha = 6;
		// tree fill color (base from which body colors are randomized)
		baseFillColor = color( 0 , 196 , 255 , fillAlpha );
		// amount of randomization from base tree fill color
		minLerpAmt = 0.5;
		maxLerpAmt = 0.8;
		// whether or not to do a random color
		randomColor = true;
		
	}
	
	// RECORD-KEEPING VARIABLES
	{
		// total mass of simulation
		overallMass = 0;
		// number of direct calculations per frame (body-body)
		directCalcCount = 0;
		// number of direct calculations per frame (body-tree)
		treeCalcCount = 0;
		// number of bodies removed due to combination
		bodiesRemoved = 0;
		// running average of time between frames
		avgFrameTime = 0;
		// number of center-of-masses drawn per frame
		comDrawn = 0;
		// maximum depth of recursive tree
		maxRecDepth = 0;
	}
}

// functions to convert linear distance and vectors from sim to window
function sim2Win( a ) {
	return a*sim2WinFactor;
}
function sim2WinVect( a ) {
	return createVector( (a.x - xMin)*sim2WinFactor ,
						 (a.y - yMin)*sim2WinFactor );
}

// CLASS Body
var Body = function( iC ) {
  // set initial position and velocity
  if( iC === 1) {
    this.x = createVector( random( xMin , xMax ) , random( yMin , yMax ) );
    this.v = createVector( 0 , 0 );
  }
  if( iC === 2) {
    this.x = p5.Vector.random2D();
    this.x.mult( random( 0 , 0.1*minExt ) );
    this.v = createVector( 0 , 0 );
  }
  if( iC === 3) {
    this.x = p5.Vector.random2D();
    this.v = createVector( this.x.y , -this.x.x );
    this.x.mult( random( 0.25*minExt , 0.3*minExt ) );
    var d = this.x.mag();
    this.v.mult( 2.0 * d );
  }
	// a = acceleration
	this.a = createVector( 0 , 0 );
	// m = mass
	this.m = random( minMass , maxMass );
	// diameter
	this.d = sqrt(this.m/avgMass)*bodyDiam;
	// c = color
	this.c = bodyColor;
	// treeLoc = location in QuadTree: array of integers
	this.treeLoc = [];
	// randomize color
	if( randomColor ) {
		var rc = color( random(0,255) , random(0,255) , random(0,255) , fillAlpha );
		var la = random( minLerpAmt , maxLerpAmt );
		this.c = lerpColor( baseFillColor , rc , la );
	}
	
	// Body: method to draw the body to the screen
	this.draw = function() {
		if( this.m > 0 ) {
			fill( bodyColor );
			var c = sim2WinVect( this.x );
			ellipse( c.x , c.y , this.d , this.d );
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
  // is it the root of the tree?
	this.isRoot = false;
	// does it have children (branches)?
	this.hasChildren = false;
	// does it contain a body (is it a leaf)?
	this.hasBody = false;
	// 2D vector of center
	this.center = center;
	// dist from center to edge in both dimensions
	this.halfDimX = halfDimX;
	this.halfDimY = halfDimY;
	this.halfDimMin = min(halfDimX , halfDimY);
	// array of QuadTrees (recursive)
	this.children = new Array(4);
	// this QuadTree's parent
	this.parent = 0;
	// this QuadTree's body
	this.body = 0;
  // depth from root
	this.depth = 0;
	// center of mass (2d Vector)
	this.com = 0;
	// total mass contained in this tree and all children
	this.totalMass = 0;
	
	// QuadTree: method to determine which child a body belongs in
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
	
	// QuadTree: method to add children to a tree
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
		

		
		if( this.depth > maxRecDepth ) { maxRecDepth = this.depth; }
	}
	
	
	// QuadTree: method to add a body to a tree (recursive)
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
					var a = new Body( initCond );
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
					// set this.body to new body
					this.body.x = createVector( a.x.x , a.x.y );
					this.body.v = createVector( a.v.x , a.v.y );
					this.body.a = createVector( 0 , 0 );
					this.body.c = a.c;
					this.body.m = a.m;
					this.body.d = sqrt(this.body.m/avgMass)*bodyDiam;
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
	
	// QuadTree: method to update center of mass and total mass for tree
	this.updateCenters = function() {
		// if there are children, call function on children,
		// then use children's center of mass to find own COM
		if( this.hasChildren ) {
			this.children[0].updateCenters();
			this.children[1].updateCenters();
			this.children[2].updateCenters();
			this.children[3].updateCenters();
			this.totalMass = this.children[0].totalMass +
							 this.children[1].totalMass +
							 this.children[2].totalMass +
							 this.children[3].totalMass ;
			var c0 = createVector( this.children[0].com.x , this.children[0].com.y );
			var c1 = createVector( this.children[1].com.x , this.children[1].com.y );
			var c2 = createVector( this.children[2].com.x , this.children[2].com.y );
			var c3 = createVector( this.children[3].com.x , this.children[3].com.y );
			c0.mult( this.children[0].totalMass );
			c1.mult( this.children[1].totalMass );
			c2.mult( this.children[2].totalMass );
			c3.mult( this.children[3].totalMass );
			c0.add( c1 );
			c0.add( c2 );
			c0.add( c3 );
			c0.mult( 1 / this.totalMass );
			this.com = createVector( c0.x , c0.y );
		}
		// if there are no children
		else {
			// if there is a body, set com to its values
			if( this.hasBody ) {
				this.com = createVector( this.body.x.x , this.body.x.y );
				this.totalMass = this.body.m;
			}
			// if not body or children, com is center of tree, and mass = 0
			else {
				this.com = createVector( this.center.x , this.center.y );
				this.totalMass = 0;
			}
		}
	};
	
	// QuadTree: method to get resultant acceleration on a body (b) from tree
	this.getResultantAcc = function( b ) {
		var x1 = createVector( b.x.x , b.x.y );
		var m1 = b.m;
		this.a = createVector( 0 , 0 );
		var d = 0;
		//console.log(this.com);
		// if tree has children, check if it is distant enough
		if( this.hasChildren ) {
			d = p5.Vector.dist( x1 , this.com );
			// if it is distant enough
			if( this.halfDimMin / d < theta ) {
				// compute acceleration based on center of mass
				x2 = createVector( this.com.x , this.com.y );
				m2 = this.totalMass;
				this.a = p5.Vector.sub( x1 , x2 );
				this.a.normalize();
				var fm = universalConstant / pow( d * d + epsilon * epsilon , 1.5 );
				this.a.mult( fm * m2 );
				treeCalcCount++;
				//console.log( "tree:  " + a );
				return this.a;
			}
			// if it is not distant enough
			else {
				// get accelerations from children
				this.a.add( this.children[0].getResultantAcc( b ) );
				this.a.add( this.children[1].getResultantAcc( b ) );
				this.a.add( this.children[2].getResultantAcc( b ) );
				this.a.add( this.children[3].getResultantAcc( b ) );
				return this.a;
			}
		}
		// if tree has no children
		else {
			// if tree has a body, compute acc based on it
			if( this.hasBody ) {
				x2 = createVector( this.body.x.x , this.body.x.y );
				m2 = this.body.m;
				d = p5.Vector.dist( x1 , x2 );
				this.a = p5.Vector.sub( x1 , x2 );
				this.a.normalize();
				var fm = universalConstant / pow( d * d + epsilon * epsilon , 1.5 );
				this.a.mult( fm * m2 );
				directCalcCount++;
				//onsole.log( "direct:  " + a );
				return this.a;
			}
			// if tree does not have a body, return zero vector
			else {
				return createVector( 0 , 0 );
			}
		}
	};
	
	// QuadTree: method to draw centers of mass
	this.drawCentersOfMass = function() {
		if( this.totalMass > comDrawThreshold ) {
			fill( comColor );
			noStroke();
			var x = sim2WinVect( this.com );
			//if( this.totalMass < overallMass*1.1 ) {
			if( true ) {
				var d = comSizeFactor*sqrt( sim2Win( this.totalMass ) );
				ellipse( x.x , x.y , d , d );
			}
			comDrawn++;
			if( this.hasChildren ) {
				this.children[0].drawCentersOfMass();
				this.children[1].drawCentersOfMass();
				this.children[2].drawCentersOfMass();
				this.children[3].drawCentersOfMass();
			}
		}
	};
	
	// QuadTree: method to draw divisions
	this.drawDiv = function() {
		if( this.hasChildren ) {
			var c = sim2WinVect( this.center );
			var hdx = sim2Win( this.halfDimX );
			var hdy = sim2Win( this.halfDimY );
			stroke( divColor );
			line( c.x - hdx , c.y , c.x + hdx , c.y );
			line( c.x , c.y - hdy , c.x , c.y + hdy );
			this.children[0].drawDiv();
			this.children[1].drawDiv();
			this.children[2].drawDiv();
			this.children[3].drawDiv();
		}
	}
	
	// QuadTree: method to color children with bodies
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
	
	// QuadTree: method to draw 3D tree (WEBGL only)
	this.draw3D = function( z ) {
	  var x = this.com.x*sim2WinFactor*0.5;
	  var y = this.com.y*sim2WinFactor*0.5;
	  vertex( x , y , z );
	  if( this.hasChildren ) {
	    this.children[0].draw3D( z + 0.1*minRes );
	    vertex( x , y , z );
	    this.children[1].draw3D( z + 0.1*minRes );
	    vertex( x , y , z );
	    this.children[2].draw3D( z + 0.1*minRes );
	    vertex( x , y , z );
	    this.children[3].draw3D( z + 0.1*minRes );
	    vertex( x , y , z );
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
		this.B[i] = new Body( initCond );
		this.T.addBody( this.B[i] );
	}
	this.T.updateCenters();
	overallMass = this.T.totalMass;
	
	
	// BodySim: method to draw the bodies
	this.drawBodies = function() {
		noStroke();
		for( var i = 0 ; i < this.N ; i++ ) {
			this.B[i].draw();
		}
	};
	
	// BodySim: method to remove bodies with zero mass
	this.removeZeroMasses = function() {
		ind = [];
		for( var i = 0 ; i < this.N ; i++ ) {
			if( (this.B[i].m === 0 ) ) {
				append( ind,  i );
				console.log( "REMOVED A BODY!" );
				bodiesRemoved++;
			}
		}
		this.N -= ind.length;
		reverse( ind );
		for( var i = 0 ; i < ind.length ; i++ ) {
			this.B.splice( ind[i] , 1 );
		}
	}
	
	// BodySim: method to draw the QuadTree divisions and color the populated children
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
	
	// BodySim: method to update the QuadTree
	this.updateTree = function() {
		this.T = createQuadTree( simCenter , 0.5*xExt , 0.5*yExt );
		// populate the quadTree
		for( var i = 0 ; i < this.N ; i++ ) {
			this.T.addBody( this.B[i] );
		}
	};
	
	// BodySim: method to zero all accelerations
	this.zeroAccelerations = function() {
		for( var i = 0 ; i < this.N ; i++ ) {
			this.B[i].a = createVector( 0 , 0 );
		}
	};
	
	// BodySim: method to apply friction forces to all bodies
	this.applyFrictionForces = function() {
		for( var i = 0 ; i < this.N ; i++ ) {
			dA = createVector( this.B[i].v.x , this.B[i].v.y );
			if( !reversePhysics ) {
				dA.mult( -frictionConstantAttract*dA.mag() );
			} else {
				dA.mult( -frictionConstantRepel*dA.mag() );
			}
			this.B[i].a.add( dA );
		}
	};
	 
	// BodySim: method to apply mutual forces to all bodies (BRUTE)
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
	
	// BodySim: method to apply mutual forces to all bodies (TREE)
	this.applyMutualForcesTree = function() {
		// do for each body
		for( var i = 0 ; i < this.N ; i++ ) {
			// get resulting acceleration from tree
			if( reversePhysics ) {
				this.B[i].a.add( this.T.getResultantAcc( this.B[i] ) );
			} else {
				this.B[i].a.sub( this.T.getResultantAcc( this.B[i] ) );
			}
		}
	};
	
	// BodySim: method to apply edge forces
	this.applyEdgeForces = function() {
		for( var i = 0 ; i < this.N ; i++ ) {
			var x = this.B[i].x.x;
			var y = this.B[i].x.y;

			if( this.B[i].x.x < xMin ) {
				this.B[i].x.x = xMin;
				this.B[i].v.x = abs( this.B[i].v.x * (1-edgeNormalDamping) );
				//this.B[i].v.x = abs( this.B[i].v.x );
				//this.B[i].v.mult( (1-edgeNormalDamping) );
			}
			if( this.B[i].x.y < yMin ) {
				this.B[i].x.y = yMin;
				this.B[i].v.y = abs( this.B[i].v.y * (1-edgeNormalDamping) );
				//this.B[i].v.y = abs( this.B[i].v.y );
				//this.B[i].v.mult( (1-edgeNormalDamping) );
			}
			if( this.B[i].x.x > xMax ) {
				this.B[i].x.x = xMax;
				this.B[i].v.x = -abs( this.B[i].v.x * (1-edgeNormalDamping) );
				//this.B[i].v.x = -abs( this.B[i].v.x );
				//this.B[i].v.mult( (1-edgeNormalDamping) );
			}
			if( this.B[i].x.y > yMax ) {
				this.B[i].x.y = yMax;
				this.B[i].v.y = -abs( this.B[i].v.y * (1-edgeNormalDamping) );
				//this.B[i].v.y = -abs( this.B[i].v.y );
				//this.B[i].v.mult( (1-edgeNormalDamping) );
			}
		}
	};
	
	// BodySim: method to evolve the simulation 1/2 step
	this.evolveHalfStep = function() {
		
		this.zeroAccelerations();
		this.updateTree();
		this.removeZeroMasses();
		this.T.updateCenters();
		if( bruteMethod ) {
			this.applyMutualForcesBrute();
		} else {
			this.applyMutualForcesTree();
		}
		this.applyEdgeForces();
		this.applyFrictionForces();
		for( var i = 0 ; i < this.N ; i++ ) {
			this.B[i].v.add( p5.Vector.mult( this.B[i].a , dt / 2 ) );
		}
	};
	
	// BodySim: method to evolve the simulation num full steps
	this.evolveFullStep = function( num ) {
		for( var n = 0 ; n < num ; n++ ) {
			for( var i = 0 ; i < this.N ; i++ ) {
				this.B[i].x.add( p5.Vector.mult( this.B[i].v , dt ) );
			}
			this.zeroAccelerations();
			this.updateTree();
			this.removeZeroMasses();
			this.T.updateCenters();
			if( bruteMethod ) {
				this.applyMutualForcesBrute();
			} else {
				this.applyMutualForcesTree();
			}
			this.applyEdgeForces();
			this.applyFrictionForces();
			for( var i = 0 ; i < this.N ; i++ ) {
				this.B[i].v.add( p5.Vector.mult( this.B[i].a , dt ) );
			}
		}
	};
	
	
	
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
	
	
	// display beginning text
	background( 0 , 0 , 0 );
	textAlign( CENTER );
	textSize( 50 );
	fill(255);
	text("N-BODY QUADTREE\n-marthematicist-" , 0.5*xRes , 0.3*yRes - 80 );
	textSize( 25 );
	if( !artMode ) {
		text( "A particle physics simulation\nutilizing the Barnes-Hut algorithm." , 0.5*xRes , 0.3*yRes + 70 );
	}
	textSize( 40 );
	text( "[Double-click(tap) to reverse physics]" , 0.5*xRes , 0.3*yRes + 190 );
	textSize( 30 );
	text( "version " + versionNumber , 0.5*xRes , yRes - 100 );
	textSize( 20 );
	if( artMode ) {
		text( "artMode" , 0.5*xRes , yRes - 60 );
	} else {
		text( "N=" + numBodies + "   field dimensions=" + round(xExt*100)/100 + "x" + round(yExt*100)/100 +
			  "   avgMass=" + round(overallMass/numBodies*100)/100  + "\nG=" + universalConstant + "   epsilon=" + epsilon + "   theta=" + theta +
			  "   dt=" + dt   , 0.5*xRes , yRes - 60 );
	}
	
	startTimer = millis();
}

function draw() {
	// record keeping
	directCalcCount = 0;
	treeCalcCount = 0;
	
	// if still in setup, don't draw anything
	if( millis() - startTimer < startWaitTime ) {
		return
	}
	if( clearFirstTime ) {
		background( 0 , 0 , 0 );
		clearFirstTime = false;
		modeChangeTimer = millis();
	}
	
	
	// draw background
	background( bgColor );
	

	// evolve the simulation full steps
	if( !mouseIsPressed ) {
		S.evolveFullStep(1);
	}
	
	/*
	// draw 3D tree
	stroke( 255 );
	translate( 0 , 0 , -minRes );
	rotateY( mouseX*0.01 + frameCount*0.01 );
	rotateX( mouseY*0.01 );
	beginShape();
	S.T.draw3D( -0.5*minRes );
	endShape();
	*/
	
	
	// draw centers of mass
	if( drawCOM ) {
		comDrawn = 0;
		S.T.drawCentersOfMass(); }
	
	// draw QuadTree
	S.drawTree( drawTreeFill , drawTreeDiv );
	
	
	// draw bodies
	if( drawBodies ) { S.drawBodies(); }
	
	
	// if mode changed recently, display mode
	if( millis() - modeChangeTimer < modeChangeDisplayTime ) {
		fill( 0 , 0 , 0 , 64 );
		noStroke();
		textAlign( CENTER );
		textSize( 30 );
		var textWidth = 450;
		var textHeight = 80;
		rect( 0.5*xRes - 0.5*textWidth , yRes - 90 , textWidth , textHeight , 10 );
		fill( 255 , 255 , 255 , 128 );
		if( millis() - modeChangeTimer < modeChangeDisplayTime*0.95 ) {
			if( !reversePhysics ) {
				text( "physics mode: ATTRACT\n[double-click/tap to reverse]" , 0.5*xRes , yRes - 60 );
			} else {
				text( "physics mode: REPEL\n[double-click/tap to reverse]" , 0.5*xRes , yRes - 60 );
			}
		}
	}
	

	avgFrameTime = avgFrameTime*0.8 + (millis() - frameTimer)*0.2;
	if( frameCount%1000 === 0 ) {
		console.log( "theta:" + theta +
					 "   direct calc:" + directCalcCount +
					 "   treeCalc:" +  treeCalcCount +
					 "   totalCalc:" + (directCalcCount+treeCalcCount) +
					 "   bodiesRemoved:" + bodiesRemoved +
					 "   avgFrameTime:" + avgFrameTime +
					 "   comDrawn=" + comDrawn);
	}
	frameTimer = millis();
}

function mousePressed() {
	if( millis() - clickTimer < doubleClickTime ) {
		reversePhysics = !reversePhysics;
		modeChangeTimer = millis();
	}
	clickTimer = millis();
}



