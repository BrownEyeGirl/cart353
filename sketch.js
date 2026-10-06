const flock = [];

function setup() {
  createCanvas(800, 600);
  canvas.parent('p5-container'); 
  
  // Create an initial group of boids
  for (let i = 0; i < 100; i++) {
    flock.push(new Boids(random(width), random(height)));
  }
}

function draw() {
  background(30);

  // Update and show each boid
  for (let boid of flock) {
    boid.edges();
    boid.flock(flock);
    boid.update();
    boid.show();
  }
}

// Add new boids by clicking and dragging the mouse
function mouseDragged() {
  flock.push(new Boids(mouseX, mouseY));
}

class Boids {
  constructor(x, y) {
    this.position = createVector(x, y);
    this.velocity = p5.Vector.random2D();
    this.velocity.setMag(random(2, 4));
    this.acceleration = createVector();
    this.maxForce = 0.2;
    this.maxSpeed = 4;
    this.r = 6; // size of the boid
  }

  // Wrap around the edges of the screen
  edges() {
    if (this.position.x > width) this.position.x = 0;
    if (this.position.x < 0) this.position.x = width;
    if (this.position.y > height) this.position.y = 0;
    if (this.position.y < 0) this.position.y = height;
  }

  // Apply forces for alignment, cohesion, and separation
  flock(boids) {
    let alignment = this.align(boids);
    let cohesion = this.cohere(boids);
    let separation = this.separate(boids);

    // Weight the forces (adjust multipliers to change behavior)
    alignment.mult(1.0);
    cohesion.mult(1.0);
    separation.mult(1.5);

    this.acceleration.add(alignment);
    this.acceleration.add(cohesion);
    this.acceleration.add(separation);
  }

  update() {
    this.velocity.add(this.acceleration);
    this.velocity.limit(this.maxSpeed);
    this.position.add(this.velocity);
    this.acceleration.mult(0); // Reset acceleration each frame
  }

  show() {
    // Draw boid as a small circle (or rotate a triangle toward velocity)
    stroke(255);
    fill(200, 100);
    ellipse(this.position.x, this.position.y, this.r * 2);
  }

  // Steering helper function
  calcSteering(target) {
    let desired = p5.Vector.sub(target, this.position);
    desired.setMag(this.maxSpeed);
    let steer = p5.Vector.sub(desired, this.velocity);
    steer.limit(this.maxForce);
    return steer;
  }

  // Rule 1: Alignment (Steer in the average direction of neighbors)
  align(boids) {
    let neighborDist = 50;
    let sum = createVector();
    let count = 0;
    for (let other of boids) {
      let d = dist(this.position.x, this.position.y, other.position.x, other.position.y);
      if (d > 0 && d < neighborDist) {
        sum.add(other.velocity);
        count++;
      }
    }
    if (count > 0) {
      sum.div(count);
      sum.setMag(this.maxSpeed);
      let steer = p5.Vector.sub(sum, this.velocity);
      steer.limit(this.maxForce);
      return steer;
    }
    return createVector();
  }

  // Rule 2: Cohesion (Steer toward the average position of neighbors)
  cohere(boids) {
    let neighborDist = 50;
    let sum = createVector();
    let count = 0;
    for (let other of boids) {
      let d = dist(this.position.x, this.position.y, other.position.x, other.position.y);
      if (d > 0 && d < neighborDist) {
        sum.add(other.position);
        count++;
      }
    }
    if (count > 0) {
      sum.div(count);
      return this.calcSteering(sum);
    }
    return createVector();
  }

  // Rule 3: Separation (Steer away from crowded neighbors)
  separate(boids) {
    let desiredSeparation = 25;
    let steer = createVector();
    let count = 0;
    for (let other of boids) {
      let d = dist(this.position.x, this.position.y, other.position.x, other.position.y);
      if (d > 0 && d < desiredSeparation) {
        let diff = p5.Vector.sub(this.position, other.position);
        diff.normalize();
        diff.div(d); // Closer boids = stronger push away
        steer.add(diff);
        count++;
      }
    }
    if (count > 0) {
      steer.div(count);
    }
    if (steer.mag() > 0) {
      steer.setMag(this.maxSpeed);
      steer.sub(this.velocity);
      steer.limit(this.maxForce);
    }
    return steer;
  }
}
