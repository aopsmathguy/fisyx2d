var f2 = {};
f2.HashGrid = class{
  obj;
  constructor(){
    this.obj = {};
  }
  key(x, y){
    return x + " " + y;
  }
  clear(){
    this.obj = {};
  }
  add(x, y, elem){
    var hash = this.key(x,y);
    if (!this.obj[hash]){
      this.obj[hash] = new Set();
    }
    this.obj[hash].add(elem);
  }
  remove(x, y, elem){
    var hash = this.key(x,y);
    if (!this.obj[hash]){
      return;
    }
    this.obj[hash].delete(elem);
  }
  get(x,y){
    var hash = this.key(x,y);
    return this.obj[hash];
  }
};
f2.Vec2 = class {
  constructor(x, y){
    this.x = x;
    this.y = y;
  }
  projX(){
    return new f2.Vec2(this.x, 0);
  }
  projY(){
    return new f2.Vec2(0, this.y);
  }
  floor(){
    return new f2.Vec2(Math.floor(this.x), Math.floor(this.y));
  }
  rotate(theta) {
    return new f2.Vec2(this.x * Math.cos(theta) - this.y * Math.sin(theta), this.y * Math.cos(theta) + this.x * Math.sin(theta));
  }
  dot(o){
    return o.x * this.x + o.y * this.y;
  }
  cross(o){
    return this.x * o.y - o.x * this.y;
  }
  crossZ(c){
    return new f2.Vec2(c * this.y, -c * this.x);
  }
  rCrossZ(c){
    return new f2.Vec2(-c * this.y, c * this.x);
  }
  magnitude() {
    return Math.sqrt(this.x * this.x + this.y * this.y);
  }
  abs(){
    return new f2.Vec2(Math.abs(this.x), Math.abs(this.y));
  }
  normalize() {
    var mag = this.magnitude();
    if (mag != 0){
      return this.multiply(1 / this.magnitude());
    }
    return new f2.Vec2(1,0);
  }
  normal(to){
    return this.subtract(to).normalize().rotate(-Math.PI/2);
  }
  multiplyV(v){
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
  copy() {
    return new f2.Vec2(this.x, this.y);
  }
  static copy(opts){
    opts = opts || {};
    return new f2.Vec2(opts.x || 0, opts.y || 0);
  }
};
f2.World = class{
  gravity = 9.81;
  scale = 20;
  gridSize = 3;

  allBodies;
  nextId;

  staticBodies;

  dynamicBodies;

  staticBodiesRegions;
  dynamicBodiesRegions;

  constraints;
  constructor(){
    this.allBodies = {};
    this.nextId = 0;

    this.staticBodies = {};

    this.dynamicBodies = {};

    this.staticBodiesRegions = new f2.HashGrid();
    this.dynamicBodiesRegions = new f2.HashGrid();

    this.constraints = {};
  }
  addConstraint(c){
      var bid1 = c.body1.id;
      var bid2 = c.body2.id;
      this.constraints[bid1] = this.constraints[bid1] || {};
      this.constraints[bid1][bid2] = this.constraints[bid1][bid2] || [];
      this.constraints[bid1][bid2].push(c);

      this.constraints[bid2] = this.constraints[bid2] || {};
      this.constraints[bid2][bid1] = this.constraints[bid2][bid1] || [];
      this.constraints[bid2][bid1].push(c);
  }
  dimensionsInMeters(){
    return (new f2.Vec2(innerWidth, innerHeight)).multiply(1/this.scale);
  }
  transform(ctx, func){
    ctx.save();
    ctx.scale(this.scale,this.scale);
    func();
    ctx.restore();
  }
  displayRect(ctx, min, max){
    ctx.lineWidth = 0.1;
    ctx.strokeStyle = "#000";
    ctx.fillStyle = "rgba(255,255,255,0)";
    var minGrid = this.getGrid(min);
    var maxGrid = this.getGrid(max);
    var idxSet = new Set();
    for (var xGrid = minGrid.x - 1; xGrid <= maxGrid.x + 1; xGrid++){
      for (var yGrid = minGrid.y - 1; yGrid <= maxGrid.y + 1; yGrid++){
        var list = this.staticBodiesRegions.get(xGrid,yGrid);
        if (list == undefined){
          continue;
        }
        list.forEach((item) => {
          idxSet.add(item);
        });
      }
    }
    idxSet.forEach((i) =>{
      this.staticBodies[i].display(ctx);
    });
    this.updateDynamicf2.HashGrid();
    for (var xGrid = minGrid.x - 1; xGrid <= maxGrid.x + 1; xGrid++){
      for (var yGrid = minGrid.y - 1; yGrid <= maxGrid.y + 1; yGrid++){
        var list = this.dynamicBodiesRegions.get(xGrid,yGrid);
        if (list == undefined){
          continue;
        }
        list.forEach((item) => {
          this.dynamicBodies[item].display(ctx);
        });
      }
    }
    for (var i in this.constraints){
        for (var j in this.constraints[i]){
            var c = this.constraints[i][j];
            for (var k = 0; k < c.length; k++){
                c[k].display(ctx);
            }
        }
    }
  }
  display(ctx){
    ctx.lineWidth = 0.1;
    ctx.strokeStyle = "#fff";
    ctx.fillStyle = "rgba(255,255,255,0)";
    for (var i in this.staticBodies){
        this.staticBodies[i].display(ctx);
    }
    for (var i in this.dynamicBodies){
        this.dynamicBodies[i].display(ctx);
    }
    for (var i in this.constraints){
        for (var j in this.constraints[i]){
            var c = this.constraints[i][j];
            for (var k = 0; k < c.length; k++){
                c[k].display(ctx);
            }
        }
    }
  }
  intersect(A, B){
    var slop = 0.03;
    var percent = 0.4;
    var beta = 10;

    var APoly = A.generateShape();
    var BPoly = B.generateShape();

    var AEdge = APoly.findAxisOfLeastPen(BPoly);
    var BEdge = BPoly.findAxisOfLeastPen(APoly);
    var edgeBody;
    var vertBody;
    var intersectInfo;
    if (AEdge.penetration > BEdge.penetration){
      edgeBody = A;
      vertBody = B;
      intersectInfo = AEdge;
    }
    else{
      edgeBody = B;
      vertBody = A;
      intersectInfo = BEdge;
    }
    var normal = intersectInfo.normal;
    var penetration = intersectInfo.penetration;
    if (penetration > -slop){
      return false;
    }
    var vertex = intersectInfo.vertex;
    var edge = intersectInfo.vertex.add(normal.multiply(-penetration));

    var rEdge = edge.subtract(edgeBody.position);
    var rVert = vertex.subtract(vertBody.position);

    f2.solvePosition(edgeBody,vertBody,rEdge,rVert,normal, percent*penetration);


    var vEdge = edgeBody.getVelocity(rEdge);
    var vVert = vertBody.getVelocity(rVert);

    var rel = vEdge.subtract(vVert);
    var normRel = normal.dot(rel);
    if (normRel < 0){
      return true;
    }

    var elasticity = f2.combinedElasticity(edgeBody.elasticity, vertBody.elasticity);
    var dvel = -(1 + elasticity) * normRel;

    var imp = f2.solveVelocity(edgeBody,vertBody,rEdge,rVert,normal, beta * penetration + dvel);
    f2.applyImpulses(edgeBody, vertBody, rEdge, rVert, normal.multiply(imp));

        var tangent = rel.subtract(normal.multiply(rel.dot(normal))).normalize();
        // var tangent = (rel.dot(normal) < 0 ? normal.crossZ(1) : normal.crossZ(-1));

    var relVelAlongTangent =  rel.dot(tangent);
    var mu = f2.combinedFrictionCoef(edgeBody.sFriction, vertBody.sFriction)
    var tImp = f2.solveVelocity(edgeBody,vertBody,rEdge,rVert,tangent, -relVelAlongTangent);
    if (Math.abs(tImp) > - imp* mu){
      var dmu = f2.combinedFrictionCoef(edgeBody.kFriction, vertBody.kFriction);
      tImp = imp * dmu;
    }
    f2.applyImpulses(edgeBody, vertBody, rEdge, rVert, tangent.multiply(tImp));
    return true;
  }
  updateDynamicHashGrid(){
    this.dynamicBodiesRegions.clear();
    for (var i in this.dynamicBodies){
        var body = this.dynamicBodies[i];
        var center = this.getGrid(body.position);
        this.dynamicBodiesRegions.add(center.x, center.y, i);
    }
  }
  getGrid(pos){
    return pos.multiply(1/this.gridSize).floor();
  }
  addBody(b){
    var id = this.nextId++;
    b.id = id;
    this.allBodies[id] = b;
    if (b.mass == Infinity){
      this.staticBodies[id] = b;
      var poly = b.generateShape();
      var minG = this.getGrid(poly.min);
      var maxG = this.getGrid(poly.max);
      for (var x = minG.x - 1; x <= maxG.x + 1; x++){
        for (var y = minG.y - 1; y <= maxG.y + 1; y++){
          this.staticBodiesRegions.add(x, y, id);
        }
      }
    }
    else{
      this.dynamicBodies[id] = b;
    }
  }
  removeBody(b){
    var id = b.id;
    delete this.allBodies[id];
    if (b.mass == Infinity){
      delete this.staticBodies[id];
      var poly = b.generateShape();
      var minG = this.getGrid(poly.min);
      var maxG = this.getGrid(poly.max);
      for (var x = minG.x - 2; x <= maxG.x + 2; x++){
        for (var y = minG.y - 2; y <= maxG.y + 2; y++){
          this.staticBodiesRegions.remove(x, y, id);
        }
      }
    }
    else{
      delete this.dynamicBodies[id];
    }
  }
  getDynamicIntersects(body){
    
    var grid = this.getGrid(body.position);
    var s = new Set();
    for (var x = grid.x - 1; x <= grid.x + 1; x++){
      for (var y = grid.y - 1; y <= grid.y + 1; y++){
        var toAdd = this.dynamicBodiesRegions.get(x, y);
        if (toAdd == undefined){
          continue;
        }
        toAdd.forEach((item) => {
          s.add(item);
        });
      }
    }
    return Array.from(s);
  }
  getStaticIntersects(body){
    var grid = this.getGrid(body.position);
    return this.staticBodiesRegions.get(grid.x, grid.y);
  }
  step(t){
    for (var i in this.dynamicBodies){
        var body = this.dynamicBodies[i];
        body.applyImpulse(new f2.Vec2(0, t * body.mass * this.gravity));
    }
    for (var i in this.dynamicBodies){
        var body = this.dynamicBodies[i];
        body.integrate(t);
    }

    var moved = new Set();
    this.updateDynamicHashGrid();
    for (var i in this.dynamicBodies){
        moved.add(i);
    }
    var count = 0;
    // while(moved.size > 0){
    for (var k = 0; k < 10 && moved.size > 0; k++){
      var newMoved = new Set();
      moved.forEach((i) => {
        var myConstraints = this.constraints[i];
        for (var j in myConstraints){
            var c = myConstraints[j];
            for (var k = 0; k < c.length; k++){
                var m = c[k].solve();
                if (m){
                    if (this.dynamicBodies[j]){
                        newMoved.add(j);
                    }
                    newMoved.add(i);
                }
            }

        }
        var body = this.dynamicBodies[i];
        var dynamicInt = this.getDynamicIntersects(body);
        for (var j in dynamicInt){
          var idx = dynamicInt[j];
          if (i == idx){
            continue;
          }
          var oBody = this.dynamicBodies[idx];
          var m = this.intersect(body, oBody);
          if (m){
            newMoved.add(i);
            newMoved.add(idx);
          }
        }
        var staticInt = this.getStaticIntersects(body);
        if (staticInt == undefined){
          return;
        }
        staticInt.forEach((j) => {
          var oBody = this.staticBodies[j];
          var m = this.intersect(body, oBody);
          if (m){
            newMoved.add(i);
          }
        });
      });
      moved = newMoved;
      count++;
    }
  }
};
f2.Circle = class{
  sType = "c";
  center;
  radius;
  min;
  max;
  constructor(center, radius){
    this.center = center;
    this.radius = radius;
    this.min = center.add(new f2.Vec2(- this.radius, - this.radius));
    this.max = center.add(new f2.Vec2(this.radius, this.radius));
  }
  display(ctx){
    ctx.save();
    ctx.beginPath();
    ctx.arc(this.center.x, this.center.y, this.radius, 0, 2 * Math.PI);
    ctx.closePath();
    ctx.stroke();
    ctx.restore();
  }
  extremePoint(dir){
    return this.center.add(dir.multiply(this.radius));
  }
  findAxisOfLeastPen(b){
    switch (b.sType){
      case "c":
        var centerDisp = b.center.subtract(this.center);
        var normal = centerDisp.normalize();
        var bestDistance = centerDisp.magnitude() - (b.radius + this.radius);
        var vertex = b.extremePoint(normal.multiply(-1));
        return {normal : normal, penetration : bestDistance, vertex : vertex};
        break;
      //circle vs polygon vertex
      case "p":
        var closestDist = 10000000;
        var vertexIdx;
        for (var i = 0; i < b.vs.length; i++){
          var v = b.vs[i];
          var dist = this.center.distanceTo(v);
          if (dist < closestDist){
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
        if (toCenter.dot(toNext) > 0 || toCenter.dot(toPrev) > 0){
          bestDistance = -10000000;
        }
        return {normal : normal, penetration : bestDistance, vertex : now};
        break;
    }
  }
};
f2.Polygon = class {
  sType = "p";
  vs;
  min;
  max;
  constructor(vs){
    this.vs = vs;
    this.min = new f2.Vec2(10000000,10000000);
    this.max = new f2.Vec2(-10000000,-10000000);
    for (var i = 0; i < this.vs.length; i++){
      var point = this.vs[i];
      this.min.x = Math.min(this.min.x, point.x);
      this.min.y = Math.min(this.min.y, point.y);
      this.max.x = Math.max(this.max.x, point.x);
      this.max.y = Math.max(this.max.y, point.y);
    }
  }
  display(ctx){
    ctx.save();
    ctx.beginPath();
    for (var i = 0; i < this.vs.length; i++){
      var point = this.vs[i];
      if (i == 0){
        ctx.moveTo(point.x, point.y);
      }
      else{
        ctx.lineTo(point.x, point.y);
      }
    }
    ctx.closePath();
    ctx.stroke();
    ctx.restore();
  }
  extremePoint(dir){
    var bestProj = - 100000000;
    var out;
    for (var i = 0; i < this.vs.length; i++)
    {
        var v = this.vs[i];
        var proj = v.dot(dir);
        if (proj > bestProj){
          out = v;
          bestProj = proj;
        }
    }
    return out;
  }
  findAxisOfLeastPen(b){
    switch (b.sType){
      //polygon edge vs circle
      case "c":
        var bestDistance = -100000000;
        var normal;
        var vertex;
        for (var i = 0; i < this.vs.length; i++){
          var from = this.vs[i];
          var to = this.vs[(i + 1) % this.vs.length];
          var n = from.normal(to);
          var s = b.extremePoint(n.multiply(-1));
          var v = this.vs[i];
          var d = n.dot(s.subtract(v));
          if (d > bestDistance){
            bestDistance = d;
            normal = n;
            vertex = s;
          }
        }
        return {normal : normal, penetration : bestDistance, vertex : vertex};
        break;
      case "p":
        var bestDistance = -100000000;
        var normal;
        var vertex;
        for (var i = 0; i < this.vs.length; i++){
          var from = this.vs[i];
          var to = this.vs[(i + 1) % this.vs.length];
          var n = from.normal(to);
          var s = b.extremePoint(n.multiply(-1));
          var v = this.vs[i];
          var d = n.dot(s.subtract(v));
          if (d > bestDistance){
            bestDistance = d;
            normal = n;
            vertex = s;
          }
        }
        return {normal : normal, penetration : bestDistance, vertex : vertex};
        break;
    }
  }
};
f2.Body = class{
  id;

  mass;
  inertia;
  kFriction;
  sFriction;
  elasticity;

  position;
  _velocity;
  velocity;

  angle;
  _angleVelocity;
  angleVelocity;
  constructor(opts){
    opts = opts || {};
    this.id = opts.id || 0;

    this.mass = opts.mass || 1;
    this.inertia = opts.inertia || 1;
    this.kFriction = opts.kFriction || 0.1;
    this.sFriction = opts.sFriction || 0.2;
    this.elasticity = opts.elasticity || 0.3;

    this.position = f2.Vec2.copy(opts.position) || new f2.Vec2(0,0);
    this.velocity = f2.Vec2.copy(opts.velocity) || new f2.Vec2(0,0);
    this._velocity = this.velocity;

    this.angle = opts.angle || 0;
    this.angleVelocity =  opts.angleVelocity || 0;
    this._angleVelocity = this.angleVelocity;
  }
  static generateBody(opts){
    if (opts.bodyType == "c"){
      return new f2.CircleBody(opts);
    }
    else if (opts.bodyType == "p"){
      return new f2.PolyBody(opts);
    }
  }
  getVelocity(r){
    return this.velocity.add(r.rCrossZ(this.angleVelocity));
  }
  getPosition(r){
    return this.position.add(r.rotate(this.angle));
  }
  applyImpulse(imp, r){
    r = r || new f2.Vec2(0,0);
    this.velocity = this.velocity.add(imp.multiply(1 / this.mass));
    this.angleVelocity += 1/this.inertia * r.cross(imp);
  }
  applyMassDisplacement(massLength, r){
    this.position = this.position.add(massLength.multiply(1 / this.mass));
    this.angle += 1/this.inertia * r.cross(massLength);
  }
  integrate(t){
    this.position = this.position.add(this.velocity.multiply(t));
    this.angle += (this.angleVelocity)* t;
    this._velocity = this.velocity;
    this._angleVelocity = this.angleVelocity;
  }
};
f2.CircleBody = class extends f2.Body{
  bodyType = "c";
  radius;
  constructor(opts){
    super(opts);
    opts = opts || {};
    this.radius = opts.radius || 1;
    this.inertia = 1/2 * this.mass * this.radius**2 || this.inertia;
  }
  display(ctx){
    ctx.save();
    ctx.translate(this.position.x,this.position.y);
    ctx.rotate(this.angle);

    ctx.beginPath();
    ctx.arc(0, 0, this.radius, 0, 2 * Math.PI);
    ctx.closePath();
    ctx.fill();
    ctx.stroke();
    ctx.moveTo(-this.radius, 0);
    ctx.lineTo(-this.radius/2, 0);
    ctx.moveTo(this.radius, 0);
    ctx.lineTo(this.radius/2, 0);
    ctx.moveTo(0, -this.radius);
    ctx.lineTo(0, -this.radius/2);
    ctx.moveTo(0, this.radius);
    ctx.lineTo(0, this.radius/2);
    ctx.stroke();

    ctx.restore();
  }
  generateShape(){
    var newCenter = this.position;
    return new f2.Circle(newCenter, this.radius);
  }
};
f2.PolyBody = class extends f2.Body{
  bodyType = "p";
  points;
  constructor(opts){
    super(opts);
    opts = opts || {};
    this.points = [];
    if (opts.points == undefined){
      this.points = [
        new f2.Vec2(1,1),
        new f2.Vec2(1,-1),
        new f2.Vec2(-1,-1),
        new f2.Vec2(-1,1)
      ];
    }else{
      for (var i = 0; i < opts.points.length; i++){
        this.points[i] = f2.Vec2.copy(opts.points[i]);
      }
    }
  }
  display(ctx){
    ctx.save();
    ctx.translate(this.position.x,this.position.y);
    ctx.rotate(this.angle);

    ctx.beginPath();
    for (var i = 0; i < this.points.length; i++){
      var point = this.points[i];
      if (i == 0){
        ctx.moveTo(point.x, point.y);
      }
      else{
        ctx.lineTo(point.x, point.y);
      }
    }
    ctx.closePath();
    ctx.fill();
    ctx.stroke();

    ctx.restore();
  }
  generateShape(){
    var newPoints = [];
    for (var i = 0; i < this.points.length; i++){
      newPoints[i] = this.points[i].rotate(this.angle).add(this.position);
    }
    return new f2.Polygon(newPoints);
  }
};
f2.RectBody = class extends f2.PolyBody{
  constructor(opts){
    super(opts);
    opts = opts || {};
    this.width = opts.width || 2;
    this.length = opts.length || 2;
    this.points = [
      new f2.Vec2(this.length/2, this.width/2),
      new f2.Vec2(this.length/2, -this.width/2),
      new f2.Vec2(-this.length/2, -this.width/2),
      new f2.Vec2(-this.length/2, this.width/2)
    ];
    this.inertia = 1/12 * this.mass * (this.length**2 + this.width**2);
  }
};
f2.Constraint = class {
    body1;
    body2;
    r1;
    r2;
    length;
    constructor(opts){
        opts = opts || {};
        this.body1 = opts.body1;
        this.body2 = opts.body2;
        this.r1 = opts.r1;
        this.r2 = opts.r2;
        this.length = opts.length || 0;
    }
    setBody1(body1, r1){
        this.body1 = body1;
        this.r1 = r1;
    }
    setBody2(body2, r2){
        this.body2 = body2;
        this.r2 = r2;
    }
    setLength(l){
        this.length = l;
    }
    solve(){
        var slop = 0.03;
        var percent = 0.4;
        var beta = 10;
        var p1 = this.body1.getPosition(this.r1);
        var p2 = this.body2.getPosition(this.r2);
        var r1 = p1.subtract(this.body1.position);
        var r2 = p2.subtract(this.body2.position);
        var normal = p1.subtract(p2).normalize();
        var v1 = this.body1.getVelocity(r1);
        var v2 = this.body2.getVelocity(r2);
        var pen = this.length - p1.subtract(p2).dot(normal);
        if (Math.abs(pen) < slop){
            return false;
        }
        f2.solvePosition(this.body1, this.body2, r1, r2, normal, percent * pen);
        var imp = f2.solveVelocity(this.body1, this.body2, r1, r2, normal, beta * pen-v1.subtract(v2).dot(normal));
        f2.applyImpulses(this.body1, this.body2, r1, r2, normal.multiply(imp));
        return true;
    }
    display(ctx){
        var p1 = this.body1.getPosition(this.r1);
        var p2 = this.body2.getPosition(this.r2);
        ctx.save();
        ctx.beginPath();
        ctx.moveTo(p1.x, p1.y);
        ctx.lineTo(p2.x, p2.y);
        ctx.stroke();
        ctx.restore();
    }
};

f2.solvePosition = function(A,B,rA,rB, n, dlength){
    var normCombinedInvMass = 1/A.mass + 1/B.mass + n.cross(rA)**2/A.inertia + n.cross(rB)**2/B.inertia;
    var massMove = (dlength)/normCombinedInvMass;
    A.applyMassDisplacement(n.multiply(massMove), rA);
    B.applyMassDisplacement(n.multiply(-massMove), rB);
    return massMove;
}
f2.solveVelocity = function(A,B,rA,rB, n, dvel){
    var normCombinedInvMass = 1/A.mass + 1/B.mass + n.cross(rA)**2/A.inertia + n.cross(rB)**2/B.inertia;
    var inertia = (dvel)/normCombinedInvMass;
    return inertia;
}
f2.applyImpulses = function(A,B, rA, rB, imp){
    A.applyImpulse(imp, rA);
    B.applyImpulse(imp.multiply(-1), rB);
}
f2.combinedElasticity = function(e1, e2){
  return Math.min(e1, e2);
};
f2.combinedFrictionCoef = function(f1, f2){
  return (f1**2 + f2**2)**0.5;
};
f2.AABBvsAABB = function(Amin, Amax, Bmin, Bmax){
  var aDim = Amax.subtract(Amin);
  var bDim = Bmax.subtract(Bmin);
  var aCenter = Amax.add(Amin).multiply(0.5);
  var bCenter = Bmax.add(Bmin).multiply(0.5);
  var distCenters = aCenter.subtract(bCenter).abs();
  var dimMean = aDim.add(bDim).multiply(0.5);
  return distCenters.x <= dimMean.x && distCenters.y <= dimMean.y;
};
