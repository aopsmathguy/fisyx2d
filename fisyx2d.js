export const f2 = {};
f2.HashGrid = class {
    obj;
    constructor() {
        this.obj = {};
    }
    key(x, y) {
        return x + " " + y;
    }
    clear() {
        this.obj = {};
    }
    add(x, y, elem) {
        var hash = this.key(x, y);
        if (!this.obj[hash]) {
            this.obj[hash] = new Set();
        }
        this.obj[hash].add(elem);
    }
    remove(x, y, elem) {
        var hash = this.key(x, y);
        if (!this.obj[hash]) {
            return;
        }
        this.obj[hash].delete(elem);
    }
    get(x, y) {
        var hash = this.key(x, y);
        return this.obj[hash];
    }
};
f2.Vec2 = class {
    constructor(x, y) {
        this.x = x;
        this.y = y;
    }
    static serialize(v){
        return {x : v.x, y : v.y};
    }
    projX() {
        return new f2.Vec2(this.x, 0);
    }
    projY() {
        return new f2.Vec2(0, this.y);
    }
    floor() {
        return new f2.Vec2(Math.floor(this.x), Math.floor(this.y));
    }
    rotate(theta) {
        return new f2.Vec2(this.x * Math.cos(theta) - this.y * Math.sin(theta), this.y * Math.cos(theta) + this.x * Math.sin(theta));
    }
    dot(o) {
        return o.x * this.x + o.y * this.y;
    }
    cross(o) {
        return this.x * o.y - o.x * this.y;
    }
    crossZ(c) {
        return new f2.Vec2(c * this.y, -c * this.x);
    }
    rCrossZ(c) {
        return new f2.Vec2(-c * this.y, c * this.x);
    }
    magnitude() {
        return Math.sqrt(this.x * this.x + this.y * this.y);
    }
    abs() {
        return new f2.Vec2(Math.abs(this.x), Math.abs(this.y));
    }
    normalize() {
        var mag = this.magnitude();
        if (mag != 0) {
            return this.multiply(1 / this.magnitude());
        }
        return new f2.Vec2(1, 0);
    }
    normal(to) {
        return this.subtract(to).normalize().rotate(-Math.PI / 2);
    }
    multiplyV(v) {
        return new f2.Vec2(this.x * v.x - this.y * v.y, this.x * v.y + this.y * v.x);
    }
    multiply(n) {
        return new f2.Vec2(this.x * n, this.y * n);
    }
    ang() {
        return Math.atan2(this.y, this.x);
    }
    add(v) {
        return new f2.Vec2(this.x + v.x, this.y + v.y);
    }
    subtract(v) {
        return new f2.Vec2(this.x - v.x, this.y - v.y);
    }
    distanceTo(v) {
        return (this.subtract(v)).magnitude();
    }
    angTo(v) {
        return (this.subtract(v)).ang();
    }
    onSegment(v1, v2) {
        var buffer = 0.001;
        return Math.min(v1.x, v2.x) - buffer <= this.x && this.x <= Math.max(v1.x, v2.x) + buffer && Math.min(v1.y, v2.y) - buffer <= this.y && this.y <= Math.max(v1.y, v2.y) + buffer;
    }
    closestToLine(v1, v2) {
        var x1 = v1.x;
        var y1 = v1.y;
        var x2 = v2.x;
        var y2 = v2.y;

        var e1x = x2 - x1;
        var e1y = y2 - y1;
        var area = e1x * e1x + e1y * e1y;
        var e2x = this.x - x1;
        var e2y = this.y - y1;
        var val = e1x * e2x + e1y * e2y;
        var on = (val > 0 && val < area);

        var lenE1 = Math.sqrt(e1x * e1x + e1y * e1y);
        var lenE2 = Math.sqrt(e2x * e2x + e2y * e2y);
        var cos = val / (lenE1 * lenE2);

        var projLen = cos * lenE2;
        var px = x1 + (projLen * e1x) / lenE1;
        var py = y1 + (projLen * e1y) / lenE1;
        return new f2.Vec2(px, py);
    }
    orientation(v1, v2) {
        var epsilon = 0.001;
        var val = (v2.y - v1.y) * (this.x - v2.x) - (v2.x - v1.x) * (this.y - v2.y);
        if (Math.abs(val) < epsilon) {
            return 0;
        }
        return (val > 0 ? 1 : -1);
    }
    static fromPolar(r, ang) {
        return new f2.Vec2(r * Math.cos(ang), r * Math.sin(ang));
    }
    copy() {
        return new f2.Vec2(this.x, this.y);
    }
    static copy(opts) {
        opts = opts || {};
        return new f2.Vec2(opts.x || 0, opts.y || 0);
    }
};

f2.World = class {
    gravity;
    gridSize;

    time;

    bodies;
    fixtures;
    nextId;

    staticFixturesRegions;
    dynamicFixturesRegions;

    contactListeners; // function(obj manifold){}

    constraints;
    constructor(opts) {
        opts = opts || {};
        this.gravity = opts.gravity || 0;
        this.gridSize = opts.gridSize || 20;

        this.time = opts.time || 0

        this.bodies = {};
        this.fixtures = {};
        this.nextId = 0;

        this.staticFixtureInd = new Set();
        this.dynamicFixtureInd = new Set();

        this.staticFixturesRegions = new f2.HashGrid();
        this.dynamicFixturesRegions = new f2.HashGrid();

        this.contactListeners = [];

        this.constraints = {};
    }
    addContactListener(f) {
        this.contactListeners.push(f);
    }
    doContactListeners(manifold) {
        for (var i = 0; i < this.contactListeners.length; i++){
            this.contactListeners[i](manifold);
        }
    }
    display(ctx, delT) {
        delT = delT || 0
        for (var i in this.bodies) {
            this.bodies[i].display(ctx, delT);
        }
    }
    getGrid(pos) {
        return pos.multiply(1 / this.gridSize).floor();
    }
    addBody(b) {
        b.world = this;
        b.id = this.nextId++;
        this.bodies[b.id] = b;
        for (var i = 0; i < b.fixtures.length; i++){
            var fix = b.fixtures[i];
            fix.id = this.nextId++;
            this.fixtures[fix.id] = fix;
            if (b.mass == Infinity){
                this.staticFixtureInd.add(fix.id);
                var min = fix.getMin();
                var max = fix.getMax();
                var minG = this.getGrid(min);
                var maxG = this.getGrid(max);
                for (var x = minG.x - 1; x <= maxG.x + 1; x++) {
                    for (var y = minG.y - 1; y <= maxG.y + 1; y++) {
                        this.staticFixturesRegions.add(x, y, fix.id);
                    }
                }
            } else{
                this.dynamicFixtureInd.add(fix.id);
            }
        }
    }
    removeBody(b) {
        b.world = undefined;
        delete this.bodies[b.id];
        b.id = undefined;
        for (var i = 0; i < b.fixtures.length; i++){
            var fix = b.fixtures[i];
            if (b.mass == Infinity){
                this.staticFixtureInd.delete(fix.id);
                var min = fix.getMin();
                var max = fix.getMax();
                var minG = this.getGrid(min);
                var maxG = this.getGrid(max);
                for (var x = minG.x - 2; x <= maxG.x + 2; x++) {
                    for (var y = minG.y - 2; y <= maxG.y + 2; y++) {
                        this.staticFixturesRegions.remove(x, y, fix.id);
                    }
                }
            } else{
                this.dynamicFixtureInd.delete(fix.id);
            }
            delete this.fixtures[fix.id];
            fix.id = undefined;
        }
    }
    addConstraint(c) {
        var id = this.nextId++;
        c.id = id;
        this.bodies[id] = c;
        this.constraints[id] = c;
    }
    removeConstraint(c) {
        var id = c.id;
        delete this.bodies[id];
        delete this.constraints[id];
    }
    updateDynamicHashGrid() {
        this.dynamicFixturesRegions.clear();
        for (var i of this.dynamicFixtureInd){
            var fix = this.fixtures[i];
            var grid_center = this.getGrid(fix.getCenter());
            this.dynamicFixturesRegions.add(grid_center.x, grid_center.y, fix.id);
        }
    }
    getDynamicIntersects(fix) {
        var grid = this.getGrid(fix.getCenter());
        var indSet = new Set();
        for (var x = grid.x - 1; x <= grid.x + 1; x++){
            for (var y = grid.y - 1; y <= grid.y + 1; y++){
                var toAdd = this.dynamicFixturesRegions.get(x, y);
                if (toAdd == undefined) {
                    continue;
                }
                toAdd.forEach((item) => {
                    indSet.add(item);
                });
            }
        }
        return indSet;
    }
    getStaticIntersects(fix) {
        var grid = this.getGrid(fix.getCenter());
        var regions = this.staticFixturesRegions.get(grid.x, grid.y);
        if (!regions) {
            return new Set();
        }
        return regions
    }
    step(t) {
        //FIX
        this.time += t
        for (var i in this.constraints) {
            var c = this.constraints[i];
            c.step(t);
        }
        for (var i in this.bodies) {
            var body = this.bodies[i];
            if (body.mass != Infinity){
                body.applyImpulse(new f2.Vec2(0, t * body.mass * this.gravity));
            }
        }
        for (var i in this.bodies) {
            var body = this.bodies[i];
            if (body.mass != Infinity){
                body.integrate(t);
            }
        }


        var moved = new Set();
        for (var i of this.dynamicFixtureInd) {
            moved.add(i);
        }
        this.updateDynamicHashGrid();
        for (var k = 0; k < 1 && moved.size > 0; k++){
            var newMovedBodies = new Set();
            moved.forEach((i)=>{
                var fixture = this.fixtures[i];
                var dynamicChecks = this.getDynamicIntersects(fixture);
                for (var j of dynamicChecks){
                    var fixture_other = this.fixtures[j];
                    if (fixture.body.id == fixture_other.body.id){
                        continue;
                    }
                    var intersectInfo = f2.Shape.intersectShapes(fixture.transformedShape, fixture_other.transformedShape);
                    if (intersectInfo.doesIntersect){
                        if (f2.Fixture.checkBitIntersect(fixture, fixture_other)){
                            var m = f2.Fixture.handleIntersectInfo(fixture, fixture_other, intersectInfo);
                            if(m.moved){
                                newMovedBodies.add(fixture.body.id);
                                newMovedBodies.add(fixture_other.body.id);
                            }
                        }
                        intersectInfo.fix1 = fixture;
                        intersectInfo.fix2 = fixture_other;
                        this.doContactListeners(intersectInfo);
                    }
                }
                var staticChecks = this.getStaticIntersects(fixture);
                for (var j of staticChecks){
                    var fixture_other = this.fixtures[j];
                    var intersectInfo = f2.Shape.intersectShapes(fixture.transformedShape, fixture_other.transformedShape);
                    if (intersectInfo.doesIntersect){
                        if (f2.Fixture.checkBitIntersect(fixture, fixture_other)){
                            var m = f2.Fixture.handleIntersectInfo(fixture, fixture_other, intersectInfo);
                            if(m.moved){
                                newMovedBodies.add(fixture.body.id);
                            }
                        }
                        this.doContactListeners(intersectInfo);
                    }
                }
            })
            var moved = new Set();
            for (var bodyId of newMovedBodies){
                var bd = this.bodies[bodyId];
                for (var fixidx = 0; fixidx < bd.fixtures.length; fixidx ++){
                    moved.add(bd.fixtures[fixidx].id);
                }
            }
        }
    }
};
f2.Shape = class {
    sType;
    constructor(sType) {
        this.sType = sType;
    }
    static serialize(p) {
        var clss = p.constructor;
        return clss.serialize(p);
    }
    static deserialize(p) {
        var clss;
        switch (p.sType) {
            case "c":
                clss = f2.Circle;
                break;
            case "p":
                clss = f2.Polygon;
                break;
        }
        return clss.deserialize(p);
    }
    static intersectShapes(APoly, BPoly) {
        var intersectInfo;
        if (!f2.AABBvsAABB(APoly.min, APoly.max, BPoly.min, BPoly.max)) {
            intersectInfo = { normal: null, penetration: 0, p2: null, p1: null };
        } else{
            var AEdge = APoly.findAxisOfLeastPen(BPoly);
            var BEdge = BPoly.findAxisOfLeastPen(APoly);
            if (AEdge.penetration > BEdge.penetration) {
                intersectInfo = AEdge;
            }
            else {
                intersectInfo = BEdge;
                intersectInfo.normal = intersectInfo.normal.multiply(-1);
                var p1 = intersectInfo.p2;
                intersectInfo.p2 = intersectInfo.p1;
                intersectInfo.p1 = p1;
            }
        }
        intersectInfo.doesIntersect = (intersectInfo.penetration < 0);
        return intersectInfo;
    }
    getAreaMoment() {
    }
}
f2.Circle = class extends f2.Shape {
    center;
    radius;
    min;
    max;
    constructor(center, radius) {
        super("c")
        this.center = f2.Vec2.copy(center);
        this.radius = radius;
        this.min = this.center.add(new f2.Vec2(- this.radius, - this.radius));
        this.max = this.center.add(new f2.Vec2(this.radius, this.radius));
    }
    static serialize(p) {
        var obj = {};
        obj.sType = p.sType;
        obj.center = p.center;
        obj.radius = p.radius;
        return obj;
    }
    static deserialize(obj) {
        return new f2.Circle(obj.center, obj.radius);
    }
    display(ctx) {
        ctx.save();
        ctx.beginPath();
        ctx.arc(this.center.x, this.center.y, this.radius, 0, 2 * Math.PI);
        ctx.closePath();
        ctx.fill();
        ctx.stroke();
        ctx.restore();
    }
    getAreaMoment() {
        var area = Math.PI * this.radius ** 2;
        var moment = Math.PI * this.radius ** 2 * (1 / 2 * this.radius ** 2 + this.center.magnitude() ** 2);
        return { area: area, moment: moment };
    }
    extremePoint(dir) {
        return this.center.add(dir.multiply(this.radius));
    }
    transform(placement) {
        return new f2.Circle(this.center.rotate(placement.angle).add(placement.position), this.radius);
    }
    findAxisOfLeastPen(b) {
        switch (b.sType) {
            case "c":
                var centerDisp = b.center.subtract(this.center);
                var normal = centerDisp.normalize();
                var bestDistance = centerDisp.magnitude() - (b.radius + this.radius);
                var vertex = b.extremePoint(normal.multiply(-1));
                return { normal: normal, penetration: bestDistance, p2: vertex, p1: vertex.add(normal.multiply(-bestDistance)) };
            //circle vs polygon vertex
            case "p":
                var closestDist = 10000000;
                var vertexIdx;
                for (var i = 0; i < b.vs.length; i++) {
                    var v = b.vs[i];
                    var dist = this.center.distanceTo(v);
                    if (dist < closestDist) {
                        closestDist = dist;
                        vertexIdx = i;
                    }
                }
                var now = b.vs[vertexIdx];
                var next = b.vs[(vertexIdx + 1) % b.vs.length];
                var prev = b.vs[(vertexIdx - 1 + b.vs.length) % b.vs.length];
                var toNext = next.subtract(now);
                var toPrev = prev.subtract(now);
                var toCenter = this.center.subtract(now);
                var normal = toCenter.multiply(-1).normalize();
                var bestDistance = -normal.dot(toCenter) - this.radius;
                if (toCenter.dot(toNext) > 0 || toCenter.dot(toPrev) > 0) {
                    bestDistance = -10000000;
                }
                return { normal: normal, penetration: bestDistance, p2: now, p1: now.add(normal.multiply(-bestDistance)) };
        }
    }
};
f2.Polygon = class extends f2.Shape {
    vs;
    min;
    max;
    constructor(vs) {
        super("p")
        this.vs = [];
        this.min = new f2.Vec2(10000000, 10000000);
        this.max = new f2.Vec2(-10000000, -10000000);
        for (var i = 0; i < vs.length; i++) {
            var point = vs[i];
            this.vs[i] = f2.Vec2.copy(point);
            this.min.x = Math.min(this.min.x, point.x);
            this.min.y = Math.min(this.min.y, point.y);
            this.max.x = Math.max(this.max.x, point.x);
            this.max.y = Math.max(this.max.y, point.y);
        }
    }
    static serialize(p) {
        var obj = {};
        obj.sType = p.sType;
        obj.vs = p.vs;
        return obj;
    }
    static deserialize(obj) {
        return new f2.Polygon(obj.vs);
    }
    display(ctx) {
        ctx.save();
        ctx.beginPath();
        for (var i = 0; i < this.vs.length; i++) {
            var point = this.vs[i];
            if (i == 0) {
                ctx.moveTo(point.x, point.y);
            }
            else {
                ctx.lineTo(point.x, point.y);
            }
        }
        ctx.closePath();
        ctx.fill();
        ctx.stroke();
        ctx.restore();
    }
    getAreaMoment() {
        var area = 0;
        var moment = 0;
        for (var i = 0; i < this.vs.length; i++) {
            var p1 = this.vs[i];
            var p2 = this.vs[(i + 1) % this.vs.length];
            area += 1 / 2 * (p1.y * p2.x - p1.x * p2.y);
            moment += 1 / 12 * (p1.y * p2.x - p1.x * p2.y)
                * (p1.x ** 2 + p2.x ** 2 + p1.x * p2.x + p1.y ** 2 + p2.y ** 2 + p1.y * p2.y)
        }
        return { area: area, moment: moment };
    }
    extremePoint(dir) {
        var bestProj = - 100000000;
        var out;
        for (var i = 0; i < this.vs.length; i++) {
            var v = this.vs[i];
            var proj = v.dot(dir);
            if (proj > bestProj) {
                out = v;
                bestProj = proj;
            }
        }
        return out;
    }
    transform(placement) {
        var vs = [];
        for (var i = 0; i < this.vs.length; i++) {
            var point = this.vs[i];
            vs[i] = point.rotate(placement.angle).add(placement.position);
        }
        return new f2.Polygon(vs);
    }
    findAxisOfLeastPen(b) {
        switch (b.sType) {
            //polygon edge vs circle
            case "c":
                var bestDistance = -100000000;
                var normal;
                var vertex;
                for (var i = 0; i < this.vs.length; i++) {
                    var from = this.vs[i];
                    var to = this.vs[(i + 1) % this.vs.length];
                    var n = from.normal(to);
                    var s = b.extremePoint(n.multiply(-1));
                    var v = this.vs[i];
                    var d = n.dot(s.subtract(v));
                    if (d > bestDistance) {
                        bestDistance = d;
                        normal = n;
                        vertex = s;
                    }
                }
                return { normal: normal, penetration: bestDistance, p2: vertex, p1: vertex.add(normal.multiply(-bestDistance)) };
                break;
            case "p":
                var bestDistance = -100000000;
                var normal;
                var vertex;
                for (var i = 0; i < this.vs.length; i++) {
                    var from = this.vs[i];
                    var to = this.vs[(i + 1) % this.vs.length];
                    var n = from.normal(to);
                    var s = b.extremePoint(n.multiply(-1));
                    var v = this.vs[i];
                    var d = n.dot(s.subtract(v));
                    if (d > bestDistance) {
                        bestDistance = d;
                        normal = n;
                        vertex = s;
                    }
                }
                return { normal: normal, penetration: bestDistance, p2: vertex, p1: vertex.add(normal.multiply(-bestDistance)) };
                break;
        }
    }
};
f2.Rectangle = class extends f2.Polygon{
    constructor(width, height, center = new f2.Vec2(0, 0)){
        super([
            center.add(new f2.Vec2(-width/2,-height/2)),
            center.add(new f2.Vec2(-width/2,height/2)),
            center.add(new f2.Vec2(width/2,height/2)),
            center.add(new f2.Vec2(width/2,-height/2))
        ])
    }
}
//TODO: write this class modify all related classes
f2.Fixture = class{
    id;
    body;

    shape;
    kFriction;
    sFriction;
    elasticity;

    categoryBits;
    maskBits;

    userData;
    transformedShape;
    constructor(opts) {
        opts = opts || {};
        this.id = undefined;
        this.body = undefined;
        this.shape = f2.Shape.deserialize(opts.shape);

        this.kFriction = isNaN(opts.kFriction) ? 0.1 : opts.kFriction;
        this.sFriction = isNaN(opts.sFriction) ? 0.1 : opts.sFriction;
        this.elasticity = isNaN(opts.elasticity) ? 0.3 : opts.elasticity;

        this.categoryBits = isNaN(opts.categoryBits) ? 0x0001 : opts.categoryBits;
        this.maskBits = isNaN(opts.maskBits) ? 0xFFFF : opts.maskBits;

        this.userData = {};
    }
    static serialize(){
        var objSerialize = {};
        objSerialize.shape = f2.Shape.serialize(this.shape);
        objSerialize.kFriction = this.kFriction;
        objSerialize.sFriction = this.sFriction;
        objSerialize.elasticity = this.elasticity;

        objSerialize.categoryBits = this.categoryBits;
        objSerialize.maskBits = this.maskBits;
        return objSerialize;
    }
    static deserialize(fix){
        return new f2.Fixture(fix);
    }
    setToBody(body){
        this.body = body;
        this.transform();
    }
    getMin(){
        return this.transformedShape.min;
    }
    getMax(){
        return this.transformedShape.max;
    }
    getCenter(){
        return new f2.Vec2((this.transformedShape.min.x + this.transformedShape.max.x)/2, (this.transformedShape.min.y + this.transformedShape.max.y)/2);
    }
    transform(){
        this.transformedShape = this.shape.transform(this.body);
        return this.transformedShape;
    }
    static checkBitIntersect(fix1, fix2){
        return (fix1.maskBits & fix2.categoryBits) != 0 && (fix2.maskBits & fix1.categoryBits) != 0;
    }
    static intersect(fix1, fix2){
        var intersectInfo = f2.Shape.intersectShapes(fix1.transformedShape, fix2.transformedShape);
        var o = f2.Fixture.handleIntersectInfo(fix1, fix2, intersectInfo);
        return o
    }
    static handleIntersectInfo(fix1, fix2, intersectInfo) {
        var A = fix1.body;
        var B = fix2.body;
        var slop = 0.01;

        var normal = intersectInfo.normal;
        var penetration = intersectInfo.penetration;
        if (penetration > -slop) {
            return {
                body1:A,
                fix1: fix1,
                body2:B,
                fix2: fix2,
                moved: false,
                collision: false,
                jnorm: 0,
                jtang: 0,
                norm: normal,
                tang: undefined
            };
        }
        var p1 = intersectInfo.p1;
        var p2 = intersectInfo.p2;

        var rA = p1.subtract(A.position);
        var rB = p2.subtract(B.position);

        f2.Body.solvePosition(A, B, rA, rB, normal, penetration);

        var vA = A.getVelocity(rA);
        var vB = B.getVelocity(rB);

        var rel = vA.subtract(vB);
        var normRel = normal.dot(rel);
        if (normRel < 0) {
            return {
                body1:A,
                fix1: fix1,
                body2:B,
                fix2: fix2,
                moved: true,
                collision: false,
                jnorm: 0,
                jtang: 0,
                norm: normal,
                tang: undefined
            };
        }

        var elasticity = f2.combinedElasticity(fix1.elasticity, fix2.elasticity);
        var dvel = -(1 + elasticity) * normRel;

        var imp = f2.Body.solveVelocity(A, B, rA, rB, normal, dvel);
        f2.Body.applyImpulses(A, B, rA, rB, normal.multiply(imp));

        var tangent = rel.subtract(normal.multiply(rel.dot(normal))).normalize();
        // var tangent = (rel.dot(normal) < 0 ? normal.crossZ(1) : normal.crossZ(-1));

        var relVelAlongTangent = rel.dot(tangent);
        var mu = f2.combinedFrictionCoef(fix1.sFriction, fix2.sFriction)
        var tImp = f2.Body.solveVelocity(A, B, rA, rB, tangent, -relVelAlongTangent);
        if (Math.abs(tImp) > - imp * mu) {
            var dmu = f2.combinedFrictionCoef(fix1.kFriction, fix2.kFriction);
            tImp = imp * dmu;
        }
        f2.Body.applyImpulses(A, B, rA, rB, tangent.multiply(tImp));
        return {
            body1:A,
            fix1: fix1,
            body2:B,
            fix2: fix2,
            moved: true,
            collision: true,
            jnorm: imp,
            jtang: tImp,
            norm: normal,
            tang: tangent
        };
    }
}

f2.Body = class {
    world;
    id;

    fixtures;

    mass;
    inertia;

    position;
    velocity;
    angle;
    angleVelocity;

    zDisplay;
    customDisplayPlacement;
    userData;

    constructor(opts) {
        opts = opts || {};

        this.world = undefined;
        this.id = undefined;

        
        this.position = f2.Vec2.copy(opts.position) || new f2.Vec2(0, 0);
        this.velocity = f2.Vec2.copy(opts.velocity) || new f2.Vec2(0, 0);

        this.angle = opts.angle || 0;
        this.angleVelocity = opts.angleVelocity || 0;

        opts.fixtures = opts.fixtures || []
        this.fixtures = [];
        for (var s = 0; s < opts.fixtures.length; s++) {
            this.addFixture(f2.Fixture.deserialize(opts.fixtures[s]));
        }

        var area = 0;
        var moment = 0;
        for (var s = 0; s < opts.fixtures.length; s++) {
            var shp = this.fixtures[s].shape;
            var am = shp.getAreaMoment();
            area += am.area;
            moment += am.moment;
        }
        this.mass = opts.mass || area;
        this.inertia = opts.inertia || this.mass / area * moment;

        this.zDisplay = opts.zDisplay || 0;
        this.customDisplayPlacement = null;
        this.userData = {};
    }
    static serialize(bd) {
        var objSerialize = {};

        objSerialize.mass = bd.mass;
        objSerialize.inertia = bd.inertia;
        objSerialize.fixtures = [];
        for (var i = 0; i < this.fixtures.length; i++){
            objSerialize.fixtures.push(f2.Fixture.serialize(this.fixtures[i]))
        }

        objSerialize.position = bd.position;
        objSerialize.velocity = bd.velocity;
        objSerialize.angle = bd.angle;
        objSerialize.angleVelocity = bd.angleVelocity;

        return objSerialize;
    }
    static serializeDynamics(bd) {
        var objSerialize = {};
        objSerialize.position = f2.Vec2.serialize(bd.position);
        objSerialize.velocity = f2.Vec2.serialize(bd.velocity);
        objSerialize.angle = bd.angle;
        objSerialize.angleVelocity = bd.angleVelocity;
        return objSerialize;
    }
    static deserialize(bd) {
        return new f2.Body(bd);
    }
    updateDynamics(opts) {
        this.moveBody(
            f2.Vec2.copy(opts.position),
            opts.angle
        );
        this.velocity = f2.Vec2.copy(opts.velocity);
        this.angleVelocity = opts.angleVelocity;
    }
    addFixture(fix){
        this.fixtures.push(fix)
        fix.setToBody(this);
    }
    createPlacement(delT) {
        return {
            position: this.position.add(this.velocity.multiply(delT)),
            angle: this.angle + this.angleVelocity * delT
        }
    }
    setCustomDisplay(f) {
        this.customDisplayPlacement = f;
    }
    display(ctx, delT) {
        delT = delT || 0;
        var plmnt = this.createPlacement(delT);
        this.displayPlacement(ctx, plmnt);
    }
    defaultDisplayPlacement(ctx, placement) {
        ctx.save();
        ctx.translate(placement.position.x, placement.position.y);
        ctx.rotate(placement.angle);
        for (var i = 0; i < this.fixtures.length; i++) {
            var s = this.fixtures[i].shape;
            s.display(ctx);
        }
        ctx.restore();
    }
    displayPlacement(ctx, placement) {
        if (this.customDisplayPlacement) {
            this.customDisplayPlacement(ctx, placement);
        }
        else {
            this.defaultDisplayPlacement(ctx, placement);
        }
    }
    moveBody(v, a) {
        this.position = v;
        this.angle = a;
        for (var i = 0; i < this.fixtures.length; i++){
            this.fixtures[i].transform()
        }
    }
    getVelocity(r) {
        return this.velocity.add(r.rCrossZ(this.angleVelocity));
    }
    getPosition(r) {
        return this.position.add(r.rotate(this.angle));
    }
    applyImpulse(imp, r) {
        r = r || new f2.Vec2(0, 0);
        this.velocity = this.velocity.add(imp.multiply(1 / this.mass));
        this.angleVelocity += 1 / this.inertia * r.cross(imp);
    }
    applyAngImpulse(t) {
        this.angleVelocity += 1 / this.inertia * t;
    }
    applyMassDisplacement(massLength, r) {
        this.moveBody(
            this.position.add(massLength.multiply(1 / this.mass)),
            this.angle + 1 / this.inertia * r.cross(massLength)
        );
        // this.position = this.position.add(massLength.multiply(1 / this.mass));
        // this.angle += 1 / this.inertia * r.cross(massLength);
    }
    static solvePosition(A, B, rA, rB, n, dlength) {
        var normCombinedInvMass = 1 / A.mass + 1 / B.mass + n.cross(rA) ** 2 / A.inertia + n.cross(rB) ** 2 / B.inertia;
        var massMove = (dlength) / normCombinedInvMass;
        A.applyMassDisplacement(n.multiply(massMove), rA);
        B.applyMassDisplacement(n.multiply(-massMove), rB);
        return massMove;
    };
    static solveVelocity(A, B, rA, rB, n, dvel) {
        var normCombinedInvMass = 1 / A.mass + 1 / B.mass + n.cross(rA) ** 2 / A.inertia + n.cross(rB) ** 2 / B.inertia;
        var inertia = (dvel) / normCombinedInvMass;
        return inertia;
    };
    static applyImpulses(A, B, rA, rB, imp) {
        A.applyImpulse(imp, rA);
        B.applyImpulse(imp.multiply(-1), rB);
    };
    integrate(t) {
        this.moveBody(
            this.position.add(this.velocity.multiply(t)),
            this.angle + (this.angleVelocity) * t
        );
        // this.position = this.position.add(this.velocity.multiply(t));
        // this.angle += (this.angleVelocity) * t;
    }
};

f2.Constraint = class {
    id;

    body1;
    r1;

    body2;
    r2;

    kconstant;
    cdamp;
    constructor(opts) {
        opts = opts || {};
        this.body1 = opts.body1;
        this.r1 = f2.Vec2.copy(opts.r1);

        this.body2 = opts.body2;
        this.r2 = f2.Vec2.copy(opts.r2);

        var r1Rot = this.r1.rotate(this.body1.angle);
        var r2Rot = this.r2.rotate(this.body2.angle);

        var p1 = this.body1.position.add(r1Rot);
        var p2 = this.body2.position.add(r2Rot);

        this.restLength = opts.restLength || p1.distanceTo(p2)
        var n = p1.subtract(p2).normalize();

        var normCombinedInvMass = 1 / this.body1.mass + 1 / this.body2.mass
            + n.cross(r1Rot) ** 2 / this.body1.inertia + n.cross(r2Rot) ** 2 / this.body2.inertia;

        this.kconstant = opts.kconstant || 1 / normCombinedInvMass;
        this.cdamp = opts.cdamp || 2 * Math.sqrt(this.kconstant / normCombinedInvMass)
    }
    step(dt) {
        var r1Rot = this.r1.rotate(this.body1.angle);
        var r2Rot = this.r2.rotate(this.body2.angle);

        var p1 = this.body1.position.add(r1Rot);
        var p2 = this.body2.position.add(r2Rot);

        var vel1 = this.body1.getVelocity(r1Rot);
        var vel2 = this.body2.getVelocity(r2Rot);

        var stretch = p1.distanceTo(p2) - this.restLength;
        var n = p1.subtract(p2).normalize();

        var relVel = vel1.subtract(vel2).dot(n);
        f2.applyImpulses(this.body1, this.body2, r1Rot, r2Rot,
            n.multiply((-this.kconstant * stretch - this.cdamp * relVel) * dt)
        );
    }
}

f2.combinedElasticity = function(e1, e2) {
    return Math.min(e1, e2);
};
f2.combinedFrictionCoef = function(f1, f2) {
    return (f1 ** 2 + f2 ** 2) ** 0.5;
};
f2.AABBvsAABB = function(Amin, Amax, Bmin, Bmax) {
    var aDim = Amax.subtract(Amin);
    var bDim = Bmax.subtract(Bmin);
    var aCenter = Amax.add(Amin).multiply(0.5);
    var bCenter = Bmax.add(Bmin).multiply(0.5);
    var distCenters = aCenter.subtract(bCenter).abs();
    var dimMean = aDim.add(bDim).multiply(0.5);
    return distCenters.x <= dimMean.x && distCenters.y <= dimMean.y;
};
