class_name XPBDCollider
extends RefCounted
## XPBD 布料碰撞体: 局部空间 球/胶囊/方箱 形状并集
## 形状定义在 root(Node3D) 局部空间 => 碰撞与视觉随 root 变换严格对齐
## 提供: 扫掠TOI(解析CCD) / 最小穿透推出 / 射线命中检测(供拖拽拾取)

var root: Node3D
var broad_r: float = 0.0                 # 外接球半径(broad phase)
var count: int = 0

var pt: PackedInt32Array = PackedInt32Array()
var poff: PackedVector3Array = PackedVector3Array()
var pa: PackedVector3Array = PackedVector3Array()
var pb: PackedVector3Array = PackedVector3Array()
var pr: PackedFloat32Array = PackedFloat32Array()
var phalf: PackedVector3Array = PackedVector3Array()
var prp: PackedFloat32Array = PackedFloat32Array()   # r+pad (begin_step 预计算)
var php: PackedVector3Array = PackedVector3Array()   # half+pad

# TOI 查询输出(免返回值免Array分配)
var ct: float = 2.0
var cn: Vector3 = Vector3.ZERO

func clear() -> void:
	pt = PackedInt32Array(); poff = PackedVector3Array()
	pa = PackedVector3Array(); pb = PackedVector3Array()
	pr = PackedFloat32Array(); phalf = PackedVector3Array()
	count = 0
	broad_r = 0.0

func add_sphere(off: Vector3, r: float) -> void:
	pt.append(0); poff.append(off); pa.append(Vector3.ZERO); pb.append(Vector3.ZERO)
	pr.append(r); phalf.append(Vector3.ZERO)
	broad_r = maxf(broad_r, off.length() + r)
	count += 1

func add_capsule(a: Vector3, b: Vector3, r: float) -> void:
	pt.append(1); poff.append(Vector3.ZERO); pa.append(a); pb.append(b)
	pr.append(r); phalf.append(Vector3.ZERO)
	broad_r = maxf(broad_r, maxf(a.length(), b.length()) + r)
	count += 1

func add_box(off: Vector3, half: Vector3) -> void:
	pt.append(2); poff.append(off); pa.append(Vector3.ZERO); pb.append(Vector3.ZERO)
	pr.append(0.0); phalf.append(half)
	broad_r = maxf(broad_r, off.length() + half.length())
	count += 1

func begin_step(pad: float) -> void:
	prp.resize(count); php.resize(count)
	for k in count:
		if pt[k] == 2:
			php[k] = phalf[k] + Vector3.ONE * pad
		else:
			prp[k] = pr[k] + pad

# 扫掠段 pre->pre+seg 与第k形状(已膨胀)最早接触: 结果写入 ct/cn (ct=2.0 未命中)
func toi_k(k: int, pre: Vector3, seg: Vector3, seg2: float) -> void:
	var ty: int = pt[k]
	if ty == 0:
		_toi_sphere(pre, seg, seg2, poff[k], prp[k])
	elif ty == 1:
		_toi_capsule(pre, seg, seg2, pa[k], pb[k], prp[k])
	else:
		_toi_box(pre, seg, poff[k], php[k])

# 最小穿透推出: 保证点在膨胀形状之外
func push_k(k: int, p: Vector3) -> Vector3:
	var ty: int = pt[k]
	if ty == 2:
		var l := p - poff[k]
		var h: Vector3 = php[k]
		if absf(l.x) <= h.x and absf(l.y) <= h.y and absf(l.z) <= h.z:
			return _box_escape(poff[k], l, h)
		return p
	var cen: Vector3 = poff[k] if ty == 0 else closest_on_seg(pa[k], pb[k], p)
	var d := p - cen
	var rr: float = prp[k]
	var ln2 := d.length_squared()
	if ln2 >= rr * rr:
		return p
	if ln2 < 1e-12:
		return cen + Vector3.UP * rr
	return cen + d * (rr / sqrt(ln2))

# 射线(ro,rn方向)命中检测: 返回最近命中距离t(局部空间单位rn), 未命中 -1
func ray_hit(ro: Vector3, rn: Vector3) -> float:
	var best := -1.0
	for k in count:
		var ty: int = pt[k]
		var t := -1.0
		if ty == 0:
			t = _ray_sphere(ro, rn, poff[k], pr[k])
		elif ty == 1:
			t = _ray_capsule(ro, rn, pa[k], pb[k], pr[k])
		else:
			t = _ray_box(ro, rn, poff[k], phalf[k])
		if t >= 0.0 and (best < 0.0 or t < best):
			best = t
	return best

# ---------------- TOI ----------------
func _toi_sphere(pre: Vector3, seg: Vector3, seg2: float, c: Vector3, rr: float) -> void:
	ct = 2.0
	var f := pre - c
	if seg2 < 1e-12:
		return
	# 线段-球心最小距离平方快筛(无开方)
	var t0 := clampf(-f.dot(seg) / seg2, 0.0, 1.0)
	if (f + seg * t0).length_squared() > rr * rr:
		return
	var b := f.dot(seg)
	var cc := f.dot(f) - rr * rr
	var t := 0.0
	if cc > 0.0:
		var disc := b * b - seg2 * cc
		t = maxf((-b - sqrt(maxf(disc, 0.0))) / seg2, 0.0)
	var nrm := f + seg * t
	var ln := nrm.length()
	ct = t
	cn = (nrm / ln) if ln > 1e-9 else Vector3.UP

func _toi_capsule(pre: Vector3, seg: Vector3, seg2: float, ax: Vector3, bx: Vector3, rr: float) -> void:
	ct = 2.0
	if seg2 < 1e-12:
		return
	var cen := closest_on_seg(ax, bx, pre)
	var f := pre - cen
	var b := f.dot(seg)
	var cc := f.dot(f) - rr * rr
	var t := 0.0
	if cc > 0.0:
		var disc := b * b - seg2 * cc
		if disc <= 0.0:
			return
		t = (-b - sqrt(disc)) / seg2
		if t < 0.0 or t > 1.0:
			return
	# 用接触时刻附近的轴向最近点作球心重解TOI, 抑制斜向擦边误差
	for k in 2:
		cen = closest_on_seg(ax, bx, pre + seg * t)
		f = pre - cen
		cc = f.dot(f) - rr * rr
		if cc <= 0.0:
			t = 0.0
			break
		b = f.dot(seg)
		var disc2 := b * b - seg2 * cc
		if disc2 <= 0.0:
			return
		var t2 := (-b - sqrt(disc2)) / seg2
		if t2 < 0.0 or t2 > 1.0:
			return
		t = t2
	ct = t
	var nrm := pre + seg * t - cen
	cn = nrm.normalized() if nrm.length_squared() > 1e-18 else Vector3.UP

# 扫掠点 vs 局部空间AABB(中心c, 半宽h含膨胀): slab法最早进入时间
func _toi_box(pre: Vector3, seg: Vector3, c: Vector3, h: Vector3) -> void:
	ct = 2.0
	var l := pre - c
	# 运动段包围盒快筛
	var end := pre + seg
	var mna := pre.min(end) - c
	var mxa := pre.max(end) - c
	if mna.x > h.x or mxa.x < -h.x or mna.y > h.y or mxa.y < -h.y or mna.z > h.z or mxa.z < -h.z:
		return
	var tmin := 0.0
	var tmax := 1.0
	var nrm := Vector3.ZERO
	for axis in 3:
		var o: float = l[axis]
		var d: float = seg[axis]
		var hw: float = h[axis]
		if absf(d) < 1e-9:
			if absf(o) > hw:
				return
			continue
		var inv := 1.0 / d
		var t1 := (-hw - o) * inv
		var t2 := (hw - o) * inv
		var enter_n := -signf(d)
		if t1 > t2:
			var tmp := t1; t1 = t2; t2 = tmp
			enter_n = signf(d)
		if t1 > tmin:
			tmin = t1
			nrm = Vector3.ZERO
			nrm[axis] = enter_n
		tmax = minf(tmax, t2)
		if tmin > tmax:
			return
	if tmin <= 0.0:
		if tmax <= 0.0:
			return
		ct = 0.0
		cn = _box_mtp_normal(l, h)
		return
	ct = tmin
	cn = nrm

# ---------------- 射线检测 ----------------
func _ray_sphere(ro: Vector3, rn: Vector3, c: Vector3, r: float) -> float:
	var f := ro - c
	var b := f.dot(rn)
	var cc := f.dot(f) - r * r
	if cc <= 0.0:
		return 0.0
	var disc := b * b - cc
	if disc < 0.0:
		return -1.0
	var t := -b - sqrt(disc)
	return t if t >= 0.0 else -1.0

func _ray_capsule(ro: Vector3, rn: Vector3, ax: Vector3, bx: Vector3, r: float) -> float:
	# 保守: 用射线对轴向线段最近点做球检测两轮细化
	var cen := closest_on_seg(ax, bx, ro + rn * 0.5)
	var t := _ray_sphere(ro, rn, cen, r)
	if t < 0.0:
		return -1.0
	cen = closest_on_seg(ax, bx, ro + rn * t)
	return _ray_sphere(ro, rn, cen, r)

func _ray_box(ro: Vector3, rn: Vector3, c: Vector3, h: Vector3) -> float:
	var l := ro - c
	var tmin := 0.0
	var tmax := 1e9
	for axis in 3:
		var o: float = l[axis]
		var d: float = rn[axis]
		var hw: float = h[axis]
		if absf(d) < 1e-9:
			if absf(o) > hw:
				return -1.0
			continue
		var inv := 1.0 / d
		var t1 := (-hw - o) * inv
		var t2 := (hw - o) * inv
		if t1 > t2:
			var tmp := t1; t1 = t2; t2 = tmp
		tmin = maxf(tmin, t1)
		tmax = minf(tmax, t2)
		if tmin > tmax:
			return -1.0
	return tmin

# ---------------- 工具 ----------------
static func closest_on_seg(a: Vector3, b: Vector3, p: Vector3) -> Vector3:
	var ab := b - a
	var t: float = clampf((p - a).dot(ab) / maxf(ab.length_squared(), 1e-12), 0.0, 1.0)
	return a + ab * t

static func _box_mtp_normal(l: Vector3, h: Vector3) -> Vector3:
	var dx := h.x - absf(l.x)
	var dy := h.y - absf(l.y)
	var dz := h.z - absf(l.z)
	var n := Vector3.ZERO
	if dx <= dy and dx <= dz:
		n.x = signf(l.x) if absf(l.x) > 1e-9 else 1.0
	elif dy <= dz:
		n.y = signf(l.y) if absf(l.y) > 1e-9 else 1.0
	else:
		n.z = signf(l.z) if absf(l.z) > 1e-9 else 1.0
	return n

static func _box_escape(c: Vector3, l: Vector3, h: Vector3) -> Vector3:
	var dx := h.x - absf(l.x)
	var dy := h.y - absf(l.y)
	var dz := h.z - absf(l.z)
	var v := l
	if dx <= dy and dx <= dz:
		v.x = (signf(l.x) if l.x != 0.0 else 1.0) * h.x
	elif dy <= dz:
		v.y = (signf(l.y) if l.y != 0.0 else 1.0) * h.y
	else:
		v.z = (signf(l.z) if l.z != 0.0 else 1.0) * h.z
	return c + v
