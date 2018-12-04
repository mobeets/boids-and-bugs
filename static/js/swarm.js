// Modified by Jay H on 2018-12-03
// Code for Boids (without controls) originally from "The Nature of Code" by Daniel Shiffman; http://natureofcode.com

var flock;
var canvas;
var canvasWidth = 500;
var canvasHeight = 400;
var nBirds = 100;
var boidSize = 2.0;
var noiseLevel = 0.0;

var doSeparation = true;
var doAlignment = true;
var doCohesion = true;
var doAvoidance = true;

// params for alignment, cohesion, and separation
var separationDist = 10;
var cohesionNeighborDist = 50;
var alignNeighborDist = 50;
var avoidanceDist = 50;
var mouseRadius = 0.5*avoidanceDist;

function windowResized() {
  resizeCanvas(canvasWidth, canvasHeight);
}

function setup() {
  canvas = createCanvas(canvasWidth, canvasHeight);
  canvas.parent('sketch-holder');  
  flock = new Flock();
  // Add an initial set of boids into the system
  for (var i = 0; i < nBirds; i++) {
    var b = new Boid(width/2,height/2);
    flock.addBoid(b);
  }
}

function drawPredator() {
  stroke('red');
  fill('red');
  strokeWeight(4);
  ellipse(mouseX, mouseY, mouseRadius);
  stroke('black');
  strokeWeight(1);
}

function draw() {
  background(51);
  flock.run();
  drawPredator();
}

// Add a new boid into the System
function mouseDragged() {
  flock.addBoid(new Boid(mouseX,mouseY));
}

// Flock object
// Does very little, simply manages the array of all the boids
function Flock() {
  // An array for all the boids
  this.boids = []; // Initialize the array
}

Flock.prototype.run = function() {
  for (var i = 0; i < this.boids.length; i++) {
    this.boids[i].run(this.boids);  // Passing the entire list of boids to each boid individually
  }
}

Flock.prototype.addBoid = function(b) {
  this.boids.push(b);
}

// Boid class
// Methods for Separation, Cohesion, Alignment added

function Boid(x,y) {
  this.acceleration = createVector(0,0);
  this.velocity = createVector(random(-1,1),random(-1,1));
  this.position = createVector(x,y);
  this.r = boidSize;
  this.maxspeed = 3;    // Maximum speed
  this.minforce = 0.01; // Minimum overall force
  this.maxforce = 0.05; // Maximum steering force
}

Boid.prototype.run = function(boids) {
  this.flock(boids);
  this.update();
  this.borders();
  this.render();
}

Boid.prototype.applyForce = function(force) {
  // We could add mass here if we want A = F / M
  this.acceleration.add(force);
}

// We accumulate a new acceleration each time based on three rules
Boid.prototype.flock = function(boids) {
  var sep = this.separate(boids);   // Separation
  var ali = this.align(boids);      // Alignment
  var coh = this.cohesion(boids);   // Cohesion
  var avo = this.avoidance(boids);   // Avoidance
  // var nse = this.noise(boids);   // Noise

  // Arbitrarily weight these forces
  sep.mult(5.0);
  ali.mult(1.0);
  coh.mult(1.0);
  avo.mult(20.0);
  // nse.mult(0.01);

  // Add the force vectors to acceleration
  if (doSeparation) {
    this.applyForce(sep);
  }
  if (doAlignment) {
    this.applyForce(ali);
  }
  if (doCohesion) {
    this.applyForce(coh);
  }
  if (doAvoidance) {
    this.applyForce(avo); 
  }
  // this.applyForce(nse);
  constrain(this.acceleration.limit, this.minforce, this.maxforce);
}

// Method to update location
Boid.prototype.update = function() {
  // Update velocity
  this.velocity.add(this.acceleration);
  // Limit speed
  this.velocity.limit(this.maxspeed);
  this.position.add(this.velocity);
  // Reset accelertion to 0 each cycle
  this.acceleration.mult(0);
}

// A method that calculates and applies a steering force towards a target
// STEER = DESIRED MINUS VELOCITY
Boid.prototype.seek = function(target) {
  var desired = p5.Vector.sub(target,this.position);  // A vector pointing from the location to the target
  // Normalize desired and scale to maximum speed
  desired.normalize();
  desired.mult(this.maxspeed);
  // Steering = Desired minus Velocity
  var steer = p5.Vector.sub(desired,this.velocity);
  steer.limit(this.maxforce);  // Limit to maximum steering force
  return steer;
}

Boid.prototype.render = function() {
  // Draw a triangle rotated in the direction of velocity
  var theta = this.velocity.heading() + radians(90);
  fill(127);
  // stroke(200);
  stroke(255, 204, 0);
  push();
  translate(this.position.x,this.position.y);
  rotate(theta);
  beginShape();
  vertex(0, -this.r*2);
  vertex(-this.r, this.r*2);
  vertex(this.r, this.r*2);
  endShape(CLOSE);
  pop();
}

// Wraparound
Boid.prototype.borders = function() {
  if (this.position.x < -this.r)  this.position.x = width +this.r;
  if (this.position.y < -this.r)  this.position.y = height+this.r;
  if (this.position.x > width +this.r) this.position.x = -this.r;
  if (this.position.y > height+this.r) this.position.y = -this.r;
}

// Avoidance
// Method steers away from cursor
Boid.prototype.avoidance = function(boids) {
  var steer = createVector(0,0);
  var count = 0;
  // For every boid in the system, check if it's too close
  mousePos = createVector(mouseX, mouseY);
  for (var i = 0; i < boids.length; i++) {
    var d = p5.Vector.dist(this.position,mousePos);
    // If the distance is greater than 0 and less than an arbitrary amount (0 when you are yourself)
    if ((d > 0) && (d < avoidanceDist)) {
      // Calculate vector pointing away from neighbor
      var diff = p5.Vector.sub(this.position, mousePos);
      diff.normalize();
      diff.div(d);        // Weight by distance
      steer.add(diff);
      count++;            // Keep track of how many
    }
  }
  // Average -- divide by how many
  if (count > 0) {
    steer.div(count);
  }

  // As long as the vector is greater than 0
  if (steer.mag() > 0) {
    // Implement Reynolds: Steering = Desired - Velocity
    steer.normalize();
    steer.mult(this.maxspeed);
    steer.sub(this.velocity);
    steer.limit(this.maxforce);
  }
  return steer;
}

// Noise
makeNoiseVector = function() {
  var vx_eps = randomGaussian(0, noiseLevel);
  var vy_eps = randomGaussian(0, noiseLevel);
  var noiseVector = createVector(vx_eps, vy_eps);
  return noiseVector;
}

// Separation
// Method checks for nearby boids and steers away
Boid.prototype.separate = function(boids) {
  var steer = createVector(0,0);
  var count = 0;
  // For every boid in the system, check if it's too close
  for (var i = 0; i < boids.length; i++) {
    var d = p5.Vector.dist(this.position,boids[i].position);
    // If the distance is greater than 0 and less than an arbitrary amount (0 when you are yourself)
    if ((d > 0) && (d < separationDist)) {
      // Calculate vector pointing away from neighbor
      var diff = p5.Vector.sub(this.position,boids[i].position);
      diff.normalize();
      diff.div(d);        // Weight by distance
      steer.add(diff);
      count++;            // Keep track of how many
    }
  }
  // Average -- divide by how many
  if (count > 0) {
    steer.div(count);
  }

  // As long as the vector is greater than 0
  if (steer.mag() > 0) {
    // Implement Reynolds: Steering = Desired - Velocity
    steer.normalize();
    steer.mult(this.maxspeed);
    steer.sub(this.velocity);
    steer.limit(this.maxforce);
  }
  return steer;
}

// Alignment
// For every nearby boid in the system, calculate the average velocity
Boid.prototype.align = function(boids) {
  var sum = createVector(0,0);
  var count = 0;
  for (var i = 0; i < boids.length; i++) {
    var d = p5.Vector.dist(this.position,boids[i].position);
    if ((d > 0) && (d < alignNeighborDist)) {
      sum.add(boids[i].velocity);
      count++;
    }
  }
  var nse = makeNoiseVector();
  sum.add(nse);

  if (count > 0) {
    sum.div(count);
    sum.normalize();
    sum.mult(this.maxspeed);
    var steer = p5.Vector.sub(sum,this.velocity);
    steer.limit(this.maxforce);
    return steer;
  } else {
    return createVector(0,0);
  }
}

// Cohesion
// For the average location (i.e. center) of all nearby boids, calculate steering vector towards that location
Boid.prototype.cohesion = function(boids) {  
  var sum = createVector(0,0);   // Start with empty vector to accumulate all locations
  var count = 0;
  for (var i = 0; i < boids.length; i++) {
    var d = p5.Vector.dist(this.position,boids[i].position);
    if ((d > 0) && (d < cohesionNeighborDist)) {
      sum.add(boids[i].position); // Add location
      count++;
    }
  }
  if (count > 0) {
    sum.div(count);
    return this.seek(sum);  // Steer towards the location
  } else {
    return createVector(0,0);
  }
}

function toggleAlignment() {
  if($(this).hasClass('active')) {
    doAlignment = true;
  } else {
    doAlignment = false;
  }
}
function toggleCohesion() {
  if($(this).hasClass('active')) {
    doCohesion = true;
  } else {
    doCohesion = false;
  }
}
function toggleSeparation() {
  if($(this).hasClass('active')) {
    doSeparation = true;
  } else {
    doSeparation = false;
  }
}

function changeNoise() {
  noiseLevel = 10*$("#slider-noise").val();
}
function changeCohesion() {
  cohesionNeighborDist = 10*$("#slider-cohesiveness").val();
}
function changeSeparation() {
  separationDist = 2*$("#slider-separation").val();
}

// to do: figure out how to change slider value
// also, make the swarm/flocking buttons and not toggles

function makeItSwarm() {
  $("#slider-noise").val(9);
  $("#slider-cohesiveness").val(9);
  changeNoise();
  changeCohesion();
  console.log("swarming");
}
function makeItFlock() {
  $("#slider-noise").val(0);
  $("#slider-cohesiveness").val(0);
  changeNoise();
  changeCohesion();
  console.log("flocking");
}

function addHandlers() {
  $("#slider-noise").click(changeNoise);
  $("#slider-cohesiveness").click(changeCohesion);
  $("#slider-separation").click(changeSeparation);
  $("#make-it-swarm").click(makeItSwarm);
  $("#make-it-flock").click(makeItFlock);
}

$(document).ready(function() {
  addHandlers();
  makeItFlock();
  $("#slider-separation").val(5);
  changeSeparation();
});
