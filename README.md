# f2.js: A 2D Physics and Math Library

This library provides a lightweight framework for 2D vector math, collision detection, rigid body dynamics, fixtures, constraints, and a basic world structure for physics simulations. The following documentation details each class, its methods, and how they interrelate. Mathematical formulas are rendered in LaTeX using dollar sign delimiters.

---

## Table of Contents

1. [HashGrid](#f2hashgrid)
2. [Vec2](#f2vec2)
3. [Shape](#f2shape)
   - [Circle](#f2circle)
   - [Polygon](#f2polygon)
   - [Rectangle](#f2rectangle)
4. [Fixture](#f2fixture)
5. [Body](#f2body)
6. [Constraint](#f2constraint)
7. [World](#f2world)
8. [Utility Functions](#f2-utilities)

---

## f2.HashGrid

**Path:** `f2.HashGrid`

This is a simple hash grid designed for efficiently grouping objects by integer-based coordinates. Internally, it uses a JavaScript `object` (`this.obj`) mapping string keys (e.g., `"x y"`) to sets of elements.

```js
f2.HashGrid = class {
    obj;
    constructor() {
        this.obj = {};
    }
    key(x, y) { ... }
    clear() { ... }
    add(x, y, elem) { ... }
    remove(x, y, elem) { ... }
    get(x, y) { ... }
};
```

### Methods

- **constructor()**  
  Initializes an empty hash grid.
  
- **key(x, y)**  
  Returns a string key in the format `x + " " + y`.
  
- **clear()**  
  Clears the entire internal storage.
  
- **add(x, y, elem)**  
  Adds an element `elem` at the grid cell `(x, y)`.
  
- **remove(x, y, elem)**  
  Removes the specified `elem` from the cell `(x, y)` if it exists.
  
- **get(x, y)**  
  Returns the `Set` of elements at grid cell `(x, y)`. If no elements exist, returns `undefined`.

Use this class in physics broad-phase checks to keep track of objects in discrete grid regions for collision detection or neighbor searches.

---

## f2.Vec2

**Path:** `f2.Vec2`

A 2D vector class providing basic vector arithmetic operations (addition, subtraction, dot products, cross products, etc.). Internally, the vector is stored in the form:

$$
\mathbf{v} = (x, y)
$$

```js
f2.Vec2 = class {
    constructor(x, y) { ... }

    static serialize(v){ ... }
    projX() { ... }
    projY() { ... }
    floor() { ... }
    rotate(theta) { ... }
    dot(o) { ... }
    cross(o) { ... }
    crossZ(c) { ... }
    rCrossZ(c) { ... }
    magnitude() { ... }
    abs() { ... }
    normalize() { ... }
    normal(to) { ... }
    multiplyV(v) { ... }
    multiply(n) { ... }
    ang() { ... }
    add(v) { ... }
    subtract(v) { ... }
    distanceTo(v) { ... }
    angTo(v) { ... }
    onSegment(v1, v2) { ... }
    closestToLine(v1, v2) { ... }
    orientation(v1, v2) { ... }
    
    static fromPolar(r, ang) { ... }
    copy() { ... }
    static copy(opts) { ... }
};
```

### Constructor

- **constructor(x, y)**  
  Creates a new vector with coordinates `(x, y)`.

### Methods

- **static serialize(v)**  
  Returns a JSON-serializable object, `{ x: v.x, y: v.y }`.

- **projX(), projY()**  
  Returns a new vector that is the projection of the current vector onto the X-axis or Y-axis respectively, by zeroing out the other component.

- **floor()**  
  Returns a new vector whose components are the floor of the current vector’s components.

- **rotate(theta)**  
  Rotates this vector by angle $ \theta $ (in radians) around the origin using the standard 2D rotation transform:  
  $$
  \begin{pmatrix}
    x' \\
    y'
  \end{pmatrix}
  =
  \begin{pmatrix}
    \cos \theta & -\sin \theta \\
    \sin \theta & \cos \theta
  \end{pmatrix}
  \begin{pmatrix}
    x \\
    y
  \end{pmatrix}.
  $$

- **dot(o)**  
  Dot product with another vector $ \mathbf{o} $.  
  $$
  \mathbf{v} \cdot \mathbf{o} = x \cdot o.x + y \cdot o.y
  $$

- **cross(o)**  
  2D cross product (which is effectively a scalar in 2D):  
  $$
  \mathbf{v} \times \mathbf{o} = x \cdot o_y - y \cdot o_x
  $$

- **crossZ(c), rCrossZ(c)**  
  Specialized cross with a scalar `c` used internally for rigid body calculations.

- **magnitude()**  
  $$
  \|\mathbf{v}\| = \sqrt{x^2 + y^2}
  $$

- **abs()**  
  Component-wise absolute value of the vector.

- **normalize()**  
  Returns a new vector in the same direction but with magnitude 1.

- **normal(to)**  
  Returns a vector normal to the line from `this` to `to`, effectively  
  $$
  [(\mathbf{v} - \mathbf{to}).\text{normalize}()].\text{rotate}\bigl(-\tfrac{\pi}{2}\bigr).
  $$

- **multiplyV(v)**  
  Treats `v` as a complex number and multiplies `(x + i y)(v.x + i v.y)`.  
  This is used for certain 2D transforms (like rotation in complex form).

- **multiply(n)**  
  Scalar multiplication.

- **ang()**  
  Returns $ \arctan2(y, x) $.

- **add(v), subtract(v)**  
  Vector addition and subtraction.

- **distanceTo(v)**  
  Distance between this vector and `v`.

- **angTo(v)**  
  Angle from this vector to `v` in polar coordinates.

- **onSegment(v1, v2)**  
  Checks if this vector is on the line segment between `v1` and `v2` within a small numerical buffer.

- **closestToLine(v1, v2)**  
  Returns the point on the line `(v1 - v2)` closest to this vector. Useful for collision computations.

- **orientation(v1, v2)**  
  Returns orientation: 0 if collinear, 1 or -1 depending on left or right turn.

- **static fromPolar(r, ang)**  
  Creates a vector from polar coordinates $(r, \text{ang})$:
  $$
  (\, r \cos(\text{ang}), \, r \sin(\text{ang}) \,)
  $$

- **copy()**, **static copy(opts)**  
  Returns a new `Vec2` duplicating this or from an object `{x, y}`.

---

## f2.Shape

**Path:** `f2.Shape`

An abstract base class for geometric shapes. Every shape has a type identifier `sType` (`"c"` for circle, `"p"` for polygon).

```js
f2.Shape = class {
    sType;
    constructor(sType) { ... }
    static serialize(p) { ... }
    static deserialize(p) { ... }
    static intersectShapes(APoly, BPoly) { ... }
    getAreaMoment() { ... }
}
```

### Notable Static Methods

- **intersectShapes(APoly, BPoly)**  
  Checks intersection between two shapes in their polygon-like forms (the library represents circles as approximate polygons internally during collision checks).

- **serialize(p), deserialize(p)**  
  For converting to and from plain objects.

### getAreaMoment()

Each shape can compute:
- **area** ($A$)
- **moment** ($I_\text{about origin}$)

used for mass-inertia computations.

---

## f2.Circle

**Path:** `f2.Circle`  
Extends **Shape** with `sType = "c"`.

```js
f2.Circle = class extends f2.Shape {
    constructor(center, radius) { ... }
    display(ctx) { ... }
    getAreaMoment() { ... }
    extremePoint(dir) { ... }
    transform(placement) { ... }
    findAxisOfLeastPen(b) { ... }
}
```

### Constructor

- **constructor(center, radius)**  
  Creates a circle at `center` (a `Vec2`), with radius `radius`.

### Methods

- **display(ctx)**  
  Basic draw method (for debugging or demonstration).
  
- **getAreaMoment()**  
  Returns an object with:
  - `area = \pi r^2`
  - `moment = \pi r^2 \left(\tfrac12 r^2 + \|\mathbf{center}\|^2\right)`  

  This is a simplified approach to compute moment about the origin. If used in a rigid body, the shape is often considered with respect to a body’s reference frame.

- **extremePoint(dir)**  
  Given a direction vector $\mathbf{dir}$, returns the farthest point on the circle along that direction, i.e.  
  $$
  \mathbf{center} + r\, \widehat{\mathbf{dir}}
  $$

- **transform(placement)**  
  Applies a rigid transform (rotation by `placement.angle` and translation by `placement.position`).

- **findAxisOfLeastPen(b)**  
  Internal collision detection routine used by the library.

---

## f2.Polygon

**Path:** `f2.Polygon`  
Extends **Shape** with `sType = "p"`.

```js
f2.Polygon = class extends f2.Shape {
    constructor(vs) { ... }
    display(ctx) { ... }
    getAreaMoment() { ... }
    extremePoint(dir) { ... }
    transform(placement) { ... }
    findAxisOfLeastPen(b) { ... }
}
```

### Constructor

- **constructor(vs)**  
  Creates a polygon from an array of `Vec2` vertices, stored in `this.vs`.

### Methods

- **display(ctx)**  
  Draws the polygon in the provided canvas context.

- **getAreaMoment()**  
  Computes the (signed) polygon area and moment about the origin using polygon area and second-moment formulas. Returns an object `{ area, moment }`.

- **extremePoint(dir)**  
  Returns the vertex with the maximum projection onto `dir`.

- **transform(placement)**  
  Rotates each vertex around the origin by `placement.angle` and then translates by `placement.position`, returning a new `Polygon`.

- **findAxisOfLeastPen(b)**  
  Used in the SAT (Separating Axis Theorem) approach to find collision penetration and normal.

---

## f2.Rectangle

**Path:** `f2.Rectangle`  
Extends **f2.Polygon**.

```js
f2.Rectangle = class extends f2.Polygon{
    constructor(width, height, center = new f2.Vec2(0, 0)) { ... }
}
```

### Constructor

- **constructor(width, height, center)**  
  Creates a rectangle of dimensions `(width, height)`, centered at `center`. Internally represented as a 4-vertex polygon.

---

## f2.Fixture

**Path:** `f2.Fixture`

A **Fixture** binds a **Shape** (Circle, Polygon, etc.) to a **Body** with additional physical properties like friction, elasticity, and collision filtering (category bits and mask bits).

```js
f2.Fixture = class{
    constructor(opts) { ... }
    static serialize() { ... }
    static deserialize(fix) { ... }
    setToBody(body) { ... }
    getMin() { ... }
    getMax() { ... }
    getCenter() { ... }
    transform() { ... }

    static checkBitIntersect(fix1, fix2) { ... }
    static intersect(fix1, fix2) { ... }
    static handleIntersectInfo(fix1, fix2, intersectInfo) { ... }
}
```

### Constructor

- **constructor(opts)**  
  - `opts.shape`: a serialized shape object to be deserialized into a `Circle`, `Polygon`, etc.  
  - `opts.kFriction`: kinetic friction coefficient ($\mu_k$).  
  - `opts.sFriction`: static friction coefficient ($\mu_s$).  
  - `opts.elasticity`: restitution factor.  
  - `opts.categoryBits, opts.maskBits`: collision filter bits.  

### Methods

- **static serialize()**, **static deserialize(fix)**  
  Serialize or deserialize a fixture.  

- **setToBody(body)**  
  Binds this fixture to a given `Body`.  

- **getMin(), getMax()**  
  Returns the AABB corners of the **transformed** shape.  

- **getCenter()**  
  Returns the midpoint `( (min_x + max_x)/2, (min_y + max_y)/2 )` of the shape in world coordinates.  

- **transform()**  
  Recomputes the shape’s world transform given the body’s current position and angle.

- **static checkBitIntersect(fix1, fix2)**  
  Returns `true` if their category and mask bits allow collision.  

- **static intersect(fix1, fix2)**  
  Calls the shape- vs.-shape intersection test and handles the collision response.  

- **static handleIntersectInfo(fix1, fix2, intersectInfo)**  
  Implements collision resolution:  
  1. Positional correction for penetration.  
  2. Velocity impulse to separate.  
  3. Application of friction if there is relative tangent motion.

---

## f2.Body

**Path:** `f2.Body`

A rigid body with position, velocity, angle, angular velocity, mass, and inertia. It can contain one or more fixtures.  

```js
f2.Body = class {
    constructor(opts) { ... }
    static serialize(bd) { ... }
    static serializeDynamics(bd) { ... }
    static deserialize(bd) { ... }
    updateDynamics(opts) { ... }
    addFixture(fix) { ... }
    createPlacement(delT) { ... }
    setCustomDisplay(f) { ... }
    display(ctx, delT) { ... }
    defaultDisplayPlacement(ctx, placement) { ... }
    displayPlacement(ctx, placement) { ... }
    moveBody(v, a) { ... }
    getVelocity(r) { ... }
    getPosition(r) { ... }
    applyImpulse(imp, r) { ... }
    applyAngImpulse(t) { ... }
    applyMassDisplacement(massLength, r) { ... }
    static solvePosition(A, B, rA, rB, n, dlength) { ... }
    static solveVelocity(A, B, rA, rB, n, dvel) { ... }
    static applyImpulses(A, B, rA, rB, imp) { ... }
    integrate(t) { ... }
};
```

### Constructor

- **constructor(opts)**  
  - `opts.position`: initial position (Vec2).  
  - `opts.velocity`: initial velocity (Vec2).  
  - `opts.angle`: initial angle (radians).  
  - `opts.angleVelocity`: initial angular velocity (radians/sec).  
  - `opts.mass`: total mass. If `Infinity`, body is treated as static.  
  - `opts.inertia`: moment of inertia about the center.  
  - `opts.fixtures`: array of serialized fixture definitions.  
  - `opts.zDisplay`: optional draw order in a custom scene.  

Internally, the body calculates mass and inertia from the sum of its fixtures (by shape areas or user override).

### Methods

- **addFixture(fix)**  
  Adds a `Fixture` to the body and sets the fixture’s `body` reference.  

- **createPlacement(delT)**  
  Predicts the body’s position at `time + delT` without modifying the actual state.

- **display(ctx, delT)**  
  Renders the body in a canvas 2D context. Optionally uses a `delT` for interpolation.

- **moveBody(v, a)**  
  Moves the body to position `v` and sets the angle to `a`. Updates all fixture transforms accordingly.

- **getVelocity(r)**  
  Returns the velocity of a point on the body offset by `r` from the body’s center. For an angular velocity $\omega$,
  $$
  \mathbf{v}(r) = \mathbf{v}_\text{cm} + \omega\, \mathbf{r}^\perp
  $$
  where $\mathbf{r}^\perp$ is $\mathbf{r}$ rotated by 90 degrees.

- **applyImpulse(imp, r)**  
  Applies an impulse $\mathbf{imp}$ at offset `r`. Updates both linear and angular velocities.

- **applyMassDisplacement(massLength, r)**  
  Used to correct positions when shapes overlap (positional correction).

- **static solvePosition(A, B, rA, rB, n, dlength)**  
  Correct positions of two bodies `A` and `B` given a normal `n` and a penetration distance.  

- **static solveVelocity(A, B, rA, rB, n, dvel)**  
  Calculates an impulse magnitude to correct velocity along a normal direction.

- **static applyImpulses(A, B, rA, rB, imp)**  
  Applies equal and opposite impulses to `A` and `B`.

- **integrate(t)**  
  Moves the body forward in time by `t`:
  $$
  \mathbf{p} \leftarrow \mathbf{p} + \mathbf{v}\, t, \quad
  \theta \leftarrow \theta + \omega\, t
  $$

---

## f2.Constraint

**Path:** `f2.Constraint`

Implements a simple spring-like constraint or distance constraint between two bodies.

```js
f2.Constraint = class {
    constructor(opts) { ... }
    step(dt) { ... }
}
```

### Constructor

- **constructor(opts)**
  - `opts.body1`, `opts.body2`: the two bodies to constrain.
  - `opts.r1`, `opts.r2`: anchor offsets relative to each body’s center.
  - `opts.restLength`: the desired distance between the anchor points.
  - `opts.kconstant`: spring constant $k$.
  - `opts.cdamp`: damping factor.

The library infers some defaults if these are omitted, e.g. computing `restLength` from the distance between anchor points at creation, or `kconstant` from the combined mass/inertia.

### step(dt)

Applies impulses to keep the bodies close to `restLength`. The force is akin to Hooke's law plus damping:
$$
F = -k\, ( \text{stretch} ) - c\, (\text{relative velocity})
$$

---

## f2.World

**Path:** `f2.World`

Manages all **Body** objects, their collisions, and constraints. It contains separate hash grids for static and dynamic fixtures.

```js
f2.World = class {
    constructor(opts) { ... }
    addContactListener(f) { ... }
    display(ctx, delT) { ... }
    addBody(b) { ... }
    removeBody(b) { ... }
    addConstraint(c) { ... }
    removeConstraint(c) { ... }
    step(t) { ... }
}
```

### Constructor

- **constructor(opts)**
  - `opts.gravity`: scalar gravitational acceleration (defaults to 0).
  - `opts.gridSize`: used for the hash grid cell size.
  - `opts.time`: initial simulation time.

Manages:
- `this.bodies`: a map of all bodies in the world.
- `this.fixtures`: references to fixtures (for broad-phase).
- `this.constraints`: constraints in the system.
- `this.contactListeners`: an array of callbacks invoked when collisions occur.

### Methods

- **addContactListener(f)**  
  Registers a function `f(obj)` that is called on collisions. The `obj` (manifold) includes collision details: `normal`, `penetration`, `doesIntersect`, and references to the fixtures.

- **display(ctx, delT)**  
  Draws all bodies at an interpolated time `time + delT`.

- **addBody(b), removeBody(b)**  
  Inserts or removes a body (and its fixtures) from the world. Static fixtures go in `staticFixturesRegions`; dynamic ones go in `dynamicFixturesRegions`.

- **addConstraint(c), removeConstraint(c)**  
  Adds or removes a constraint to the world.

- **step(t)**  
  Advances the world by `t` seconds:
  1. Updates constraints.
  2. Applies gravity to dynamic bodies.
  3. Integrates body motion.
  4. Updates the dynamic fixtures’ hash grid.
  5. Performs collision detection and resolution (up to a certain iteration count).
  6. Calls `contactListeners` for each collision.

---

## f2 Utilities

These free functions appear at the bottom of the file:

```js
f2.combinedElasticity = function(e1, e2) { ... }
f2.combinedFrictionCoef = function(f1, f2) { ... }
f2.AABBvsAABB = function(Amin, Amax, Bmin, Bmax) { ... }
```

- **combinedElasticity(e1, e2)**  
  Returns the minimum of two restitution (elasticity) coefficients.

- **combinedFrictionCoef(f1, f2)**  
  Returns  
  $$
  \sqrt{f1^2 + f2^2}
  $$  

- **AABBvsAABB(Amin, Amax, Bmin, Bmax)**  
  Checks if two axis-aligned bounding boxes (AABB) overlap by comparing their center distances to half their dimensions.

---

# Usage Example

Below is a very simple usage snippet to illustrate how you might create a world, add a body with a circular fixture, and step the simulation:

```js
import { f2 } from "./f2.js";

// Create a world with gravity = 9.8 m/s^2
const world = new f2.World({ gravity: 9.8, gridSize: 20 });

// Create a body
const myBody = new f2.Body({
  position: {x: 0, y: 0},
  mass: 1,
  fixtures: [
    {
      shape: { sType: "c", center: { x: 0, y: 0 }, radius: 1 },
      kFriction: 0.1,
      sFriction: 0.2,
      elasticity: 0.3
    }
  ]
});

// Add body to the world
world.addBody(myBody);

// Add a contact listener
world.addContactListener((info) => {
  if (info.doesIntersect) {
    console.log("Collision detected:", info);
  }
});

// In your animation loop or main update
function update() {
  // Step the physics by (e.g.) 1/60 of a second
  world.step(1/60);

  // Render
  // For instance, using canvas:
  // const ctx = canvas.getContext("2d");
  // ctx.clearRect(0, 0, canvas.width, canvas.height);
  // world.display(ctx);

  requestAnimationFrame(update);
}
update();
```

In this example:
1. We create a `World` object with standard Earth gravity.
2. We create a `Body` containing a single `Circle` fixture of radius 1.
3. We add the body to the world, register a contact listener, and start an update loop.  

Use your own rendering logic as needed. The library only provides basic canvas drawing for demonstration; you can customize or replace it as desired.

---

# Summary

- **`f2.Vec2`** provides 2D vector operations.  
- **`f2.Shape`** and its subclasses (**`Circle`**, **`Polygon`**, **`Rectangle`**) handle geometry and AABBs.  
- **`f2.Fixture`** attaches shapes to bodies with friction, elasticity, and collision filtering.  
- **`f2.Body`** is a rigid body with position, angle, velocity, and inertia.  
- **`f2.Constraint`** can simulate springs or distance constraints.  
- **`f2.World`** manages bodies, constraints, collision detection, and time stepping.  

This library is intended for 2D physics simulations, providing a straightforward but flexible approach to handle collisions, friction, restitution, and constraints in your JavaScript projects.
