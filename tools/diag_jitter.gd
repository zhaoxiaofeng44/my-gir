extends SceneTree
# 链条物理专项验收（只看导引链）：
#   A) 静止抖动：结算后连续采样 40 帧，链关节的最大逐帧位移（无风时应≈0）
#   B) 严格穿透：射线奇偶判定导引链关节在头/身 mesh 内的个数与深度
#   C) 快速转头扫掠：头骨猛转 60°，逐帧统计穿透最深值（考验"丝毫不能穿模"）
var sk: Skeleton3D
var hp: Node
var tris := []
var grid := {}
var cell := 0.05

func build_tris(mi: MeshInstance3D) -> void:
	var skin: Skin = mi.skin
	var nb: int = mini(sk.get_bone_count(), skin.get_bind_count())
	var mats := []
	mats.resize(nb)
	for b in nb:
		mats[b] = sk.get_bone_global_rest(b) * skin.get_bind_pose(b)
	for surf in mi.mesh.get_surface_count():
		var arr: Array = mi.mesh.surface_get_arrays(surf)
		var v: PackedVector3Array = arr[Mesh.ARRAY_VERTEX]
		var idx: PackedInt32Array = arr[Mesh.ARRAY_INDEX]
		var bo: PackedInt32Array = arr[Mesh.ARRAY_BONES]
		var we: PackedFloat32Array = arr[Mesh.ARRAY_WEIGHTS]
		var wp := PackedVector3Array(); wp.resize(v.size())
		for i in v.size():
			var acc := Vector3.ZERO; var ws := 0.0
			for k in 4:
				var w: float = we[i*4+k]
				if w <= 0.0: continue
				var bi: int = bo[i*4+k]
				if bi >= nb: continue
				acc += (mats[bi] as Transform3D) * v[i] * w; ws += w
			wp[i] = acc / ws if ws > 0.0 else v[i]
		var t := 0
		while t + 2 < idx.size():
			var p0: Vector3 = wp[idx[t]]; var p1: Vector3 = wp[idx[t+1]]; var p2: Vector3 = wp[idx[t+2]]
			t += 3
			if maxf(p0.y, maxf(p1.y, p2.y)) < 1.0: continue
			tris.append([p0, p1, p2])
			var mn: Vector3 = p0.min(p1).min(p2); var mx: Vector3 = p0.max(p1).max(p2)
			for gx in range(int(floor(mn.x/cell)), int(floor(mx.x/cell))+1):
				for gy in range(int(floor(mn.y/cell)), int(floor(mx.y/cell))+1):
					for gz in range(int(floor(mn.z/cell)), int(floor(mx.z/cell))+1):
						var key := gx*1000003 + gy*10007 + gz
						if not grid.has(key): grid[key] = []
						(grid[key] as Array).append(tris.size()-1)

func build_grid() -> void: pass  # 已在 build_tris 里顺带建好

var d := Vector3(1.0, 0.371, 0.193).normalized()

func ray_inside(p: Vector3) -> bool:
	var cnt := 0
	var cx := int(floor(p.x/cell)); var cy := int(floor(p.y/cell)); var cz := int(floor(p.z/cell))
	for dx in range(-2, 3):
		for dy in range(-2, 3):
			for dz in range(-2, 3):
				var key := (cx+dx)*1000003 + (cy+dy)*10007 + (cz+dz)
				if not grid.has(key): continue
				for i in (grid[key] as Array):
					var tr: Array = tris[i]
					var a: Vector3 = tr[0]
					var e1: Vector3 = tr[1]-a; var e2: Vector3 = tr[2]-a
					var h := d.cross(e2)
					var det := e1.dot(h)
					if absf(det) < 1e-12: continue
					var inv := 1.0/det
					var s := p - a
					var u := s.dot(h)*inv
					if u < 0.0 or u > 1.0: continue
					var q := s.cross(e1)
					var vv := d.dot(q)*inv
					if vv < 0.0 or u+vv > 1.0: continue
					if e2.dot(q)*inv > 1e-6: cnt += 1
	return (cnt % 2) == 1

func depth(p: Vector3) -> float:
	var best := 1e9
	var cx := int(floor(p.x/cell)); var cy := int(floor(p.y/cell)); var cz := int(floor(p.z/cell))
	for dx in range(-3, 4):
		for dy in range(-3, 4):
			for dz in range(-3, 4):
				var key := (cx+dx)*1000003 + (cy+dy)*10007 + (cz+dz)
				if not grid.has(key): continue
				for i in (grid[key] as Array):
					var tr: Array = tris[i]
					best = minf(best, _tri_dist(p, tr[0], tr[1], tr[2]))
	return best

func _tri_dist(p: Vector3, a: Vector3, b: Vector3, c: Vector3) -> float:
	var ab := b-a; var ac := c-a; var ap := p-a
	var d1 := ab.dot(ap); var d2 := ac.dot(ap)
	if d1 <= 0.0 and d2 <= 0.0: return ap.length()
	var bp := p-b
	var d3 := ab.dot(bp); var d4 := ac.dot(bp)
	if d3 >= 0.0 and d4 <= d3: return bp.length()
	var vc := d1*d4 - d3*d2
	if vc <= 0.0 and d1 >= 0.0 and d3 <= 0.0: return (ap - ab*(d1/(d1-d3))).length()
	var cp := p-c
	var d5 := ab.dot(cp); var d6 := ac.dot(cp)
	if d6 >= 0.0 and d5 <= d6: return cp.length()
	var vb := d5*d2 - d1*d6
	if vb <= 0.0 and d2 >= 0.0 and d6 <= 0.0: return (ap - ac*(d2/(d2-d6))).length()
	var va := d3*d6 - d5*d4
	if va <= 0.0 and (d4-d3) >= 0.0 and (d5-d6) >= 0.0:
		return (p - (b + (c-b)*((d4-d3)/((d4-d3)+(d5-d6))))).length()
	var den := 1.0/(va+vb+vc)
	return (p - (a + ab*(vb*den) + ac*(vc*den))).length()

func free_joints() -> Array:
	var out := []
	for ci in hp._guides.size():
		var c: int = hp._guides[ci]
		var p: PackedVector3Array = hp._chains[c]["p"]
		var na: int = hp._attach[c] if c < hp._attach.size() else 1
		for j in range(na, p.size()):
			out.append(p[j])
	return out

func root_joints() -> Array:
	var out := []
	for ci in hp._guides.size():
		var c: int = hp._guides[ci]
		var p: PackedVector3Array = hp._chains[c]["p"]
		var na: int = hp._attach[c] if c < hp._attach.size() else 1
		for j in range(na):
			out.append(p[j])
	return out

func pen_report(tag: String) -> void:
	var n := 0; var worst := 0.0; var total := 0
	for p in free_joints():
		total += 1
		if ray_inside(p):
			n += 1
			worst = maxf(worst, depth(p))
	var rn := 0; var rworst := 0.0
	for p in root_joints():
		if ray_inside(p):
			rn += 1
			rworst = maxf(rworst, depth(p))
	print("J%s 穿透：自由关节 %d 个在 mesh 内（最深 %.1f mm）｜刚性发根段 %d 个在内（最深 %.1f mm）" % [
		tag, n, worst*1000.0, rn, rworst*1000.0])

func _init():
	var scene: Node = (load("res://character_creator.tscn") as PackedScene).instantiate()
	get_root().add_child(scene)
	await process_frame
	await process_frame
	for n in scene.find_children("*", "Skeleton3D", true, false):
		if n.get_bone_count() > 200: sk = n
	hp = scene.get_node_or_null("HairPhysics")
	hp.wind = Vector3.ZERO
	for i in 260: await physics_frame   # 头发从原形态垂落到贴身需要 >200 帧，先等它落稳
	for c in sk.get_children():
		if c is MeshInstance3D and c.name in ["head", "body"]:
			build_tris(c)
	print("JT 三角形 %d" % tris.size())
	# A) 静止抖动
	var prev: Array = free_joints()
	var maxj := 0.0; var sumj := 0.0
	for f in 40:
		await physics_frame
		var cur: Array = free_joints()
		for i in cur.size():
			var d2v: float = cur[i].distance_to(prev[i])
			maxj = maxf(maxj, d2v); sumj += d2v
		prev = cur
	print("J1 静止抖动：40 帧最大逐帧位移 %.3f mm，平均 %.4f mm（应≈0）" % [maxj*1000.0, sumj*1000.0/float(40*prev.size())])
	# 拉伸量：段长偏离静止段长的最大绝对值（"链条没有弹性"必须≈0）
	var stretch := 0.0
	var stretch_pct := 0.0
	for ci in hp._guides.size():
		var c: int = hp._guides[ci]
		var pp: PackedVector3Array = hp._chains[c]["p"]
		var ll: PackedFloat32Array = hp._chains[c]["len"]
		for j in pp.size() - 1:
			var d := pp[j].distance_to(pp[j+1])
			var err := absf(d - ll[j])
			if err > stretch:
				stretch = err
				stretch_pct = err / maxf(ll[j], 1e-6) * 100.0
	print("J1b 链长拉伸：最大 %.3f mm（%.3f%%）—— 无弹性要求应≈0" % [stretch*1000.0, stretch_pct])
	pen_report("2")
	# C) 快速转头扫掠：头骨 pose 直接猛转
	var hb: int = sk.find_bone("cf_J_Head_s")
	var swing_worst := 0.0
	var swing_in := 0
	var restp: Transform3D = sk.get_bone_global_pose(hb)
	for f in 30:
		# 注意：在**固定基准**上乘旋转（在当前 pose 上累乘会让头转 5 圈，~40°/帧）
		var ang := lerpf(0.0, 1.2, float(f) / 29.0)
		sk.set_bone_global_pose(hb, restp * Transform3D(Basis(Vector3.UP, ang), Vector3.ZERO))
		await physics_frame
		var n2 := 0
		for p in free_joints():
			if ray_inside(p):
				n2 += 1
				swing_worst = maxf(swing_worst, depth(p))
		swing_in = maxi(swing_in, n2)
	print("J3 快速转头 69°：过程中在 mesh 内的关节最多 %d 个，最深 %.1f mm（必须 0）" % [swing_in, swing_worst*1000.0])
	for i in 60: await physics_frame
	pen_report("4")
	print("J_END")
	quit()
