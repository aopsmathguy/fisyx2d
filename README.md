Below is a comprehensive documentation for the **f2** library. This library provides a 2D physics engine in JavaScript with vector math utilities, shapes (circle/polygon), rigid bodies, a spatial hash (for quick lookups), constraints (like a spring), collision detection/resolution, and more.

---

## Table of Contents

1. [Overview](#overview)
2. [Classes](#classes)
   - [f2.HashGrid](#f2hashgrid)
   - [f2.Vec2](#f2vec2)
   - [f2.World](#f2world)
   - [f2.Shape](#f2shape) (abstract)
     - [f2.Circle](#f2circle)
     - [f2.Polygon](#f2polygon)
   - [f2.Body](#f2body) (base)
     - [f2.CircleBody](#f2circlebody)
     - [f2.PolyBody](#f2polybody)
     - [f2.RectBody](#f2rectbody)
   - [f2.Constraint](#f2constraint)
3. [Static Functions](#static-functions)
   - [f2.intersectShapes(A, B)](#f2intersectshapesa-b)
   - [f2.handleIntersectInfo(A, B, intersectInfo)](#f2handleintersectinfoa-b-intersectinfo)
   - [f2.intersect(A, B)](#f2intersecta-b)
   - [f2.solvePosition(A, B, rA, rB, n, dlength)](#f2solvepositiona-b-ra-rb-n-dlength)
   - [f2.solveVelocity(A, B, rA, rB, n, dvel)](#f2solvevelocitya-b-ra-rb-n-dvel)
   - [f2.applyImpulses(A, B, rA, rB, imp)](#f2applyimpulsesa-b-ra-rb-imp)
   - [f2.combinedElasticity(e1, e2)](#f2combinedelasticitye1-e2)
   - [f2.combinedFrictionCoef(f1, f2)](#f2combinedfrictioncoeff1-f2)
   - [f2.AABBvsAABB(Amin, Amax, Bmin, Bmax)](#f2aabbvsaabbamin-amax-bmin-bmax)
   - [f2.stringify(obj)](#f2stringifyobj)
   - [f2.parse(str)](#f2parsestr)
4. [Example Usage](#example-usage)
5. [License](#license)

---

## Overview

The **f2** library is a lightweight 2D physics library offering:

- **Vector operations** (via `f2.Vec2`)
- **Basic shapes** (`f2.Circle`, `f2.Polygon`)
- **Rigid bodies** (`f2.Body` and its subclasses)
- **Collision detection & resolution** (SAT-based logic and impulse resolution)
- **Spatial partitioning** (HashGrid for broad-phase optimization)
- **Constraints** (simple spring-like constraint)
- **Serialization** (save/load states)

---

## Classes

### f2.HashGrid

A simple hash grid for partitioning objects in 2D space. Uses `(x, y)` integer hashing to store sets of objects. Useful for broad-phase collision or region queries.

```js
f2.HashGrid = class {
    constructor() { ... }
    key(x, y) { ... }
    clear() { ... }
    add(x, y, elem) { ... }
    remove(x, y, elem) { ... }
    get(x, y) { ... }
};
```

**Methods**:

- **constructor()**
  - Initializes an empty `obj` map (using JavaScript objects of type `Record<string, Set<any>>`).
  
- **key(x, y): string**
  - Returns a string hash key (e.g., `"x y"`).
  
- **clear(): void**
  - Clears all entries from the hash grid.
  
- **add(x, y, elem): void**
  - Adds `elem` to the set at hash `(x, y)`.
  - Creates a new `Set` if none exists at that key.
  
- **remove(x, y, elem): void**
  - Removes `elem` from the set at hash `(x, y)`, if present.
  
- **get(x, y): Set<any> | undefined**
  - Retrieves the set of items stored at hash `(x, y)`.
  - Returns `undefined` if no set exists for that cell.

---

### f2.Vec2

A 2D vector class with numerous vector operations (addition, subtraction, rotation, dot/cross product, etc.).

```js
f2.Vec2 = class {
    constructor(x, y) { ... }
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

**Constructor**:

- **constructor(x: number, y: number)**
  - Creates a new `Vec2` with coordinates `(x, y)`.

**Instance methods**:

- **projX(): Vec2**
  - Returns the vector projected onto the X-axis, i.e., `(x, 0)`.
  
- **projY(): Vec2**
  - Returns the vector projected onto the Y-axis, i.e., `(0, y)`.

- **floor(): Vec2**
  - Returns a new `Vec2` with `Math.floor(x)` and `Math.floor(y)`.

- **rotate(theta: number): Vec2**
  - Returns a new vector rotated by `theta` radians around the origin.

- **dot(o: Vec2): number**
  - Dot product with another vector `o`.

- **cross(o: Vec2): number**
  - 2D cross product result, treated as a scalar in 2D: `this.x * o.y - o.x * this.y`.

- **crossZ(c: number): Vec2**
  - Crosses this vector with a scalar `c` treated as a z-component, returning a perpendicular vector:  
    \[
      \text{crossZ}(c) = (c * y, -c * x).
    \]

- **rCrossZ(c: number): Vec2**
  - Reverse cross with `c`: \[
      \text{rCrossZ}(c) = (-c * y, c * x).
    \]

- **magnitude(): number**
  - Returns the Euclidean length of this vector.

- **abs(): Vec2**
  - Returns a vector of absolute values: `(abs(x), abs(y))`.

- **normalize(): Vec2**
  - Returns a unit vector in the same direction. If magnitude is `0`, returns `(1,0)`.

- **normal(to: Vec2): Vec2**
  - A convenience to find a normalized normal vector from `this` to `to`, then rotate -90°.

- **multiplyV(v: Vec2): Vec2**
  - A complex multiply operation for 2D vectors re-interpreted as complex numbers:  
    \[
      (x, y) \times (v.x, v.y) = (x * v.x - y * v.y, x * v.y + y * v.x).
    \]

- **multiply(n: number): Vec2**
  - Scalar multiplication by `n`.

- **ang(): number**
  - Returns the angle of this vector in radians relative to the positive x-axis (`atan2(y, x)`).

- **add(v: Vec2): Vec2**
  - Vector addition.

- **subtract(v: Vec2): Vec2**
  - Vector subtraction.

- **distanceTo(v: Vec2): number**
  - Euclidean distance to another vector `v`.

- **angTo(v: Vec2): number**
  - Angle from `v` to this vector (`(this - v).ang()`).

- **onSegment(v1: Vec2, v2: Vec2): boolean**
  - Checks if this point lies on the line segment from `v1` to `v2` (with a small buffer).

- **closestToLine(v1: Vec2, v2: Vec2): Vec2**
  - Returns the closest point on the line `(v1, v2)` to this vector. Does not clamp to the segment.

- **orientation(v1: Vec2, v2: Vec2): number**
  - Computes orientation of `this` w.r.t line `(v1 -> v2)`.  
  - `0` if collinear (within `epsilon`), `1` if to one side, `-1` if to the other side.

- **copy(): Vec2**
  - Returns a shallow copy of this vector.

**Static methods**:

- **Vec2.fromPolar(r: number, ang: number): Vec2**
  - Creates a vector from a polar coordinate `(r, ang)`.

- **Vec2.copy(opts: { x?: number, y?: number }): Vec2**
  - Creates a `Vec2` from an object with optional `x` and `y` fields (defaults to `(0,0)` if missing).

---

### f2.World

Manages all bodies (both dynamic and static), constraints, collision filtering, and the simulation step.

```js
f2.World = class {
    constructor(opts) { ... }
    setContactFilter(f) { ... }
    setContactListener(f) { ... }
    doContactFilter(manifold) { ... }
    doContactListener(manifold) { ... }
    dimensionsInMeters() { ... }
    transform(ctx, func) { ... }
    displayRect(ctx, min, max, delT) { ... }
    display(ctx, delT) { ... }
    updateDynamicHashGrid() { ... }
    getGrid(pos) { ... }
    addBody(b) { ... }
    removeBody(b) { ... }
    addConstraint(c) { ... }
    removeConstraint(c) { ... }
    getDynamicIntersects(body) { ... }
    getStaticIntersects(body) { ... }
    step(t) { ... }
};
```

**Constructor**:

- **constructor(opts?: object)**
  - **gravity**: (default `0`) gravity in the Y-direction.
  - **scale**: (default `20`) a scale factor for rendering (pixels per meter).
  - **gridSize**: (default `20`) grid cell size for broad-phase partitioning.
  - **time**: (default `0`) simulation time tracker.

- It initializes:
  - `allBodies`, `staticBodies`, `dynamicBodies` (maps from ID to body).
  - `staticBodiesRegions`, `dynamicBodiesRegions` (`f2.HashGrid`s for each).
  - `nextId` for generating unique body IDs.
  - `constraints` map for constraints.

**Methods**:

- **setContactFilter(f: (obj: {A, B}) => boolean)**
  - Sets a function that can filter collisions. If it returns `false`, collision is ignored.

- **setContactListener(f: (obj: collisionData) => void)**
  - Sets a function that is called on valid collisions after contact resolution.

- **doContactFilter(manifold)**
  - Internally calls the set contact filter if present.

- **doContactListener(manifold)**
  - Internally calls the set contact listener if present.

- **dimensionsInMeters(): Vec2**
  - Utility for computing `(innerWidth, innerHeight) / scale`. (Browser-specific usage.)

- **transform(ctx: CanvasRenderingContext2D, func: () => void)**
  - Saves canvas context, scales by `this.scale`, calls `func()`, then restores.
  - Convenient for drawing in "physics meter" units.

- **displayRect(ctx, min, max, delT=0)**
  - Efficiently displays only static/dynamic bodies in the rectangular region `(min, max)`.
  - Uses the hash grid to find relevant bodies.

- **display(ctx, delT=0)**
  - Renders all static and dynamic bodies to the canvas.

- **updateDynamicHashGrid()**
  - Clears and repopulates `dynamicBodiesRegions` with all dynamic bodies based on their positions.

- **getGrid(pos: Vec2): Vec2**
  - Converts a position to integer grid coordinates by dividing by `gridSize` and flooring.

- **addBody(b: f2.Body)**
  - Assigns a unique ID, stores it in `allBodies`.
  - If `b.mass == Infinity`, treats it as static, storing in `staticBodies` & corresponding hash regions.
  - Otherwise, stores in `dynamicBodies`.

- **removeBody(b: f2.Body)**
  - Removes from `allBodies`, and from `staticBodies` or `dynamicBodies`.
  - Cleans up the hash grid references.

- **addConstraint(c: f2.Constraint)**
  - Creates a unique constraint ID, stores it in `constraints`.

- **removeConstraint(c: f2.Constraint)**
  - Removes from `constraints`.

- **getDynamicIntersects(body: f2.Body): Array<string>**
  - Returns the set of dynamic body IDs near `body` based on the hash grid.

- **getStaticIntersects(body: f2.Body): Array<string>**
  - Returns the set of static body IDs near `body` based on the hash grid.

- **step(t: number)**
  - Advances the simulation by time `t`.
  - Increments `this.time`.
  - Calls each constraint's `step(t)` to apply forces/impulses.
  - Applies gravity impulses to each dynamic body.
  - Integrates each dynamic body (position, velocity).
  - Repeats collision checks (broad-phase with hash grid, then narrow-phase) up to 10 iterations or until no more movement from collisions.

---

### f2.Shape (abstract)

Abstract base class for shapes. Contains static serialization/deserialization utilities.

```js
f2.Shape = class {
    constructor(sType) { ... }
    static serialize(p) { ... }
    static deserialize(p) { ... }
    getAreaMoment() { ... }
};
```

**Properties**:

- **sType**: a string identifying shape type. For example, `"c"` for circle, `"p"` for polygon.

**Methods**:

- **getAreaMoment(): { area: number, moment: number }**
  - Each subclass overrides this to compute area and second moment of area (used in body mass/inertia calculations).

- **static serialize(p: Shape): object**
  - Serializes a shape to a plain object.

- **static deserialize(p: object): f2.Shape**
  - Deserializes an object to the correct shape subclass (`f2.Circle` or `f2.Polygon`).

---

#### f2.Circle

Represents a circle shape with a center and radius.

```js
f2.Circle = class extends f2.Shape {
    constructor(center, radius) { ... }
    static serialize(p) { ... }
    static deserialize(obj) { ... }
    display(ctx) { ... }
    getAreaMoment() { ... }
    extremePoint(dir) { ... }
    transform(placement) { ... }
    findAxisOfLeastPen(b) { ... }
};
```

**Constructor**:

- **constructor(center: Vec2, radius: number)**
  - `center` is the circle center, `radius` is the circle radius.
  - Automatically sets `min` and `max` bounding box.

**Methods**:

- **static serialize(p: f2.Circle): object**
- **static deserialize(obj: any): f2.Circle**
  - Used for saving/loading.

- **display(ctx: CanvasRenderingContext2D)**
  - Draws a filled circle on the canvas.

- **getAreaMoment(): { area: number, moment: number }**
  - Computes:
    \[
      \text{area} = \pi r^2
    \]
    \[
      \text{moment} = \pi r^2 \left(\frac{1}{2}r^2 + \|\text{center}\|^2\right)
    \]

- **extremePoint(dir: Vec2): Vec2**
  - Returns the point on the circle furthest in direction `dir`.

- **transform(placement: {position: Vec2, angle: number}): f2.Circle**
  - Returns a new circle whose center is rotated by `placement.angle` and translated by `placement.position`. Radius remains unchanged.

- **findAxisOfLeastPen(b: f2.Shape): object**
  - Implementation of the "Supporting-edge vs vertex" approach for collision. Returns a structure with:
    - `normal` (penetration normal),
    - `penetration` (penetration distance),
    - `p1`, `p2` contact points.

---

#### f2.Polygon

Represents a polygon shape defined by an array of vertices (`vs`).

```js
f2.Polygon = class extends f2.Shape {
    constructor(vs) { ... }
    static serialize(p) { ... }
    static deserialize(obj) { ... }
    display(ctx) { ... }
    getAreaMoment() { ... }
    extremePoint(dir) { ... }
    transform(placement) { ... }
    findAxisOfLeastPen(b) { ... }
};
```

**Constructor**:

- **constructor(vs: Vec2[])**
  - Stores vertices. Also computes `min` and `max` bounding box.

**Methods**:

- **static serialize(p: f2.Polygon): object**
- **static deserialize(obj: any): f2.Polygon**
  - For saving/loading.

- **display(ctx: CanvasRenderingContext2D)**
  - Draws a filled polygon.

- **getAreaMoment(): { area: number, moment: number }**
  - Uses polygon area/inertia formula:
    \[
      \text{area} = \frac12 \sum (p_i \times p_{i+1})
    \]
    \[
      \text{moment} = \frac{1}{12} \sum (p_i \times p_{i+1})(\dots)
    \]
  - Summation over edges.

- **extremePoint(dir: Vec2): Vec2**
  - Returns the vertex with the largest dot product in direction `dir`.

- **transform(placement: {position: Vec2, angle: number}): f2.Polygon**
  - Returns a new polygon with all vertices rotated and translated.

- **findAxisOfLeastPen(b: f2.Shape): object**
  - Finds the axis (edge normal) of least penetration for polygon vs circle or polygon vs polygon collisions.

---

### f2.Body

Represents a rigid body composed of one or more shapes. The body has mass, inertia, friction coefficients, elasticity, position, velocity, angle, and angular velocity.

```js
f2.Body = class {
    constructor(opts) { ... }
    static serialize(bd) { ... }
    static serializeDynamics(bd) { ... }
    static deserialize(bd) { ... }
    updateDynamics(opts) { ... }
    setUserData(k, v) { ... }
    getUserData(k) { ... }
    createPlacement(delT) { ... }
    setCustomDisplayPlacement(f) { ... }
    display(ctx, delT) { ... }
    defaultDisplayPlacement(ctx, placement) { ... }
    displayPlacement(ctx, placement) { ... }
    generateShapes() { ... }
    getMinMax() { ... }
    getVelocity(r) { ... }
    getPosition(r) { ... }
    applyImpulse(imp, r) { ... }
    applyAngImpulse(t) { ... }
    applyMassDisplacement(massLength, r) { ... }
    integrate(t) { ... }
};
```

**Constructor**: 

- **constructor(opts?: object)**
  - `shapes`: array of serialized shape objects. Each shape is deserialized and stored.
  - `mass`: total mass (defaults to area if not provided).
  - `inertia`: moment of inertia (defaults computed from shapes).
  - `kFriction`, `sFriction`: kinetic and static friction coefficients (defaults to 0.1).
  - `elasticity`: restitution coefficient (defaults to 0.3).
  - `position`: initial position (Vec2).
  - `velocity`: initial velocity (Vec2).
  - `angle`: initial orientation (radians).
  - `angleVelocity`: initial angular velocity (radians/sec).
  - If `mass` is set to `Infinity`, the body is considered static (unmovable).

**Properties**:

- `shapes`: array of `f2.Shape` objects.
- `mass`, `inertia`, `kFriction`, `sFriction`, `elasticity`.
- `position`, `velocity`, `angle`, `angleVelocity`.
- `_velocity`, `_angleVelocity`: used internally to keep track of velocity states before integration.
- `customDisplayPlacement`: optional function for custom drawing.
- `userData`: an object for user-defined data.

**Methods**:

- **static serialize(bd: f2.Body): object**
  - Returns a plain object with shapes, mass, inertia, friction, elasticity, position, velocity, angle, angleVelocity.
  
- **static serializeDynamics(bd: f2.Body): object**
  - Returns only the dynamic state (`position`, `velocity`, `angle`, `angleVelocity`).

- **static deserialize(obj: any): f2.Body**
  - Constructs a new `f2.Body` from a plain object.

- **updateDynamics(opts: { position, velocity, angle, angleVelocity })**
  - Updates the dynamic parameters of the body with the given values.

- **setUserData(k: string, v: any)**
  - Attaches user-defined data to the body.

- **getUserData(k: string): any**
  - Retrieves user-defined data.

- **createPlacement(delT: number): { position: Vec2, angle: number }**
  - Predicts the next position/orientation given `delT` but does not update the body.

- **setCustomDisplayPlacement(f: (ctx, placement) => void)**
  - Assigns a custom function for drawing the body.

- **display(ctx, delT=0)**
  - By default, calculates the predicted placement and calls `displayPlacement`.

- **defaultDisplayPlacement(ctx, placement)**
  - Default method to draw the shapes (translates and rotates the canvas, draws each shape).

- **displayPlacement(ctx, placement)**
  - Dispatches to `customDisplayPlacement` if provided, otherwise uses the default.

- **generateShapes(): f2.Shape[]**
  - Transforms each local shape by the current body transform and returns them.

- **getMinMax(): { min: Vec2, max: Vec2 }**
  - Returns the combined bounding box in world coordinates, by generating shapes.

- **getVelocity(r: Vec2): Vec2**
  - Velocity at a point offset `r` from the body center, including angular velocity:  
    \[
      \text{velocity}(r) = this.velocity + rCrossZ(\text{angleVelocity}).
    \]

- **getPosition(r: Vec2): Vec2**
  - Position of a local offset `r` in world coordinates.

- **applyImpulse(imp: Vec2, r: Vec2)**
  - Immediately applies a linear + angular impulse to the body.

- **applyAngImpulse(t: number)**
  - Immediately applies an angular impulse (torque-like).

- **applyMassDisplacement(massLength: Vec2, r: Vec2)**
  - Adjusts position and angle to resolve interpenetration (positional correction).

- **integrate(t: number)**
  - Integrates linear and angular velocities over time `t`, updating position/angle.

---

#### f2.CircleBody

A convenience subclass of `f2.Body` that creates a single `f2.Circle` shape.

```js
f2.CircleBody = class extends f2.Body {
    constructor(opts) { ... }
};
```

- **constructor(opts)**
  - Expects `opts.radius` and sets up `shapes: [new f2.Circle(...)]`.

---

#### f2.PolyBody

A convenience subclass of `f2.Body` that creates a single `f2.Polygon` shape.

```js
f2.PolyBody = class extends f2.Body {
    constructor(opts) { ... }
};
```

- **constructor(opts)**
  - Expects `opts.points` (array of vertices).
  - Creates one polygon shape from them.

---

#### f2.RectBody

A rectangle convenience subclass. Inherits from `f2.PolyBody`.

```js
f2.RectBody = class extends f2.PolyBody {
    constructor(opts) { ... }
};
```

- **constructor(opts)**
  - Expects `opts.length` (X dimension) and `opts.width` (Y dimension).
  - Automatically creates a rectangle polygon centered on the origin with half extents.

---

### f2.Constraint

Implements a simple spring-like constraint between two bodies at offsets `r1` and `r2`.

```js
f2.Constraint = class {
    constructor(opts) { ... }
    step(dt) { ... }
};
```

**Constructor**:

- **constructor(opts: { body1, r1, body2, r2, restLength?, kconstant?, cdamp? })**
  - `body1`, `body2`: the two bodies to connect.
  - `r1`, `r2`: anchor points in each body’s local space.
  - `restLength`: the desired distance between anchors. Defaults to the distance at the time of creation.
  - `kconstant`: the spring constant. Defaults to a value based on inverse mass.
  - `cdamp`: damping factor. Defaults to `2 * sqrt(kconstant / massFactor)` for critical damping.

**Methods**:

- **step(dt: number)**
  - Called each time step.  
  - Computes the current anchor positions in world space, the relative velocity, and applies impulses to maintain/rest the spring at `restLength`, with damping.

---

## Static Functions

### f2.intersectShapes(A, B)

```js
f2.intersectShapes = function(APoly, BPoly) { ... }
```

- Performs a narrow-phase check (SAT or specialized circle edge check) between two **already transformed** shapes (`f2.Circle` or `f2.Polygon`).
- Returns an object `{ normal, penetration, p1, p2 }` describing the axis of least penetration, or a default with zero penetration if there's no overlap.

### f2.handleIntersectInfo(A, B, intersectInfo)

```js
f2.handleIntersectInfo = function(A, B, intersectInfo) { ... }
```

- Takes intersect info from `f2.intersectShapes` and resolves the collision if there is penetration.
- Returns an object describing the collision resolution (impulses applied, whether it moved, etc.).

### f2.intersect(A, B)

```js
f2.intersect = function(A, B) { ... }
```

- Convenience method that:
  1. Transforms the local shapes of `A` and `B` to world coordinates.
  2. For each shape pair, calls `f2.intersectShapes`.
  3. Passes the results to `f2.handleIntersectInfo`.
- Returns `{ moved, collisions }` where:
  - `moved` is true if an impulse-based correction occurred.
  - `collisions` is an array of collision records (one per shape pair that actually collided).

### f2.solvePosition(A, B, rA, rB, n, dlength)

```js
f2.solvePosition = function(A, B, rA, rB, n, dlength) { ... }
```

- Solves positional correction for interpenetration using the relative inverse mass (linear and angular).
- `dlength` is the penetration distance along `n`.
- Returns the scalar amount of displacement applied along `n`.

### f2.solveVelocity(A, B, rA, rB, n, dvel)

```js
f2.solveVelocity = function(A, B, rA, rB, n, dvel) { ... }
```

- Given a relative velocity delta `dvel` along the normal `n`, computes the required impulse that satisfies the constraints of mass + inertia.

### f2.applyImpulses(A, B, rA, rB, imp)

```js
f2.applyImpulses = function(A, B, rA, rB, imp) { ... }
```

- Applies the impulse vector `imp` at `rA` to body `A` and the opposite impulse at `rB` to body `B`.

### f2.combinedElasticity(e1, e2)

```js
f2.combinedElasticity = function(e1, e2) { ... }
```

- Returns `Math.min(e1, e2)` – the combined coefficient of restitution.

### f2.combinedFrictionCoef(f1, f2)

```js
f2.combinedFrictionCoef = function(f1, f2) { ... }
```

- Returns \(\sqrt{f1^2 + f2^2}\) – a common approach to combining friction.

### f2.AABBvsAABB(Amin, Amax, Bmin, Bmax)

```js
f2.AABBvsAABB = function(Amin, Amax, Bmin, Bmax) { ... }
```

- Quick bounding box overlap test. 
- Returns `true` if `Amin->Amax` overlaps with `Bmin->Bmax`.

### f2.stringify(obj)

```js
f2.stringify = function(obj) { ... }
```

- A JSON serializer that converts `Infinity` to a special string token `"Infinity2374852783457827"`.

### f2.parse(str)

```js
f2.parse = function(str) { ... }
```

- Reverses `f2.stringify`, converting the special string token back to `Infinity`.

---

## Example Usage

```js
// Create a physics world
let world = new f2.World({ gravity: 9.8, scale: 30, gridSize: 5 });

// Create a static ground
let ground = new f2.RectBody({
  length: 10,
  width: 1,
  mass: Infinity,        // infinite mass => static body
  position: new f2.Vec2(0, -3)
});
world.addBody(ground);

// Create a falling circle
let circle = new f2.CircleBody({
  radius: 0.5,
  position: new f2.Vec2(0, 5),
  mass: 1
});
world.addBody(circle);

// Step the simulation
function animate() {
  // Step with a fixed timestep
  world.step(1/60);

  // Clear the canvas
  ctx.clearRect(0, 0, canvas.width, canvas.height);

  // Draw everything
  world.transform(ctx, () => {
    world.display(ctx);
  });

  requestAnimationFrame(animate);
}

animate();
```

---

## License

The **f2** library does not contain explicit license text in the snippet provided. You may include or apply your own licensing terms as needed. If this code is part of a larger project, please refer to that project’s license.
