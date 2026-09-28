extends SceneTree
# 严格判据：直接用「射线奇偶法」判定链条关节是否在头/身 mesh 内部，
# 并用最近三角形给出穿透深度。不含任何椭球近似。
var sk: Skeleton3D
var hp: Node
var tris := []          # 每个元素是 [a,b,c]（骨架局部空间 = 链条所在空间）
var cell := 0.05
var tris_head := []
var tris_body := []
var grid := {}          # 三角形空间哈希

func build_tris(mi: MeshInstance3D, is_head: bool) -> void:
	var skin: Skin = mi.skin
	var mats := []
	mats.resize(sk.get_bone_count())
	for b in sk.get_bone_count():
		mats[b] = sk.get_bone_global_rest(b) * skin.get_bind_pose(b)
	for surf in mi.mesh.get_surface_count():
		var a: Array = mi.mesh.surface_get_arrays(surf)
		var v: PackedVector3Array = a[Mesh.ARRAY_VERTEX]
		var idx: PackedInt32Array = a[Mesh.ARRAY_INDEX]
		var bo: PackedInt32Array = a[Mesh.ARRAY_BONES]
		var we: PackedFloat32Array = a[Mesh.ARRAY_WEIGHTS]
		var wp := PackedVector3Array(); wp.resize(v.size())
		for i in v.size():
			var acc := Vector3.ZERO
			var ws := 0.0
			for k in 4:
				var w: float = we[i * 4 + k]
				if w <= 0.0: continue
				acc += (mats[bo[i * 4 + k]] as Transform3D) * v[i] * w
				ws += w
			wp[i] = acc / ws if ws > 0.0 else v[i]
		var t := 0
		while t + 2 < idx.size():
			var p0: Vector3 = wp[idx[t]]
			var p1: Vector3 = wp[idx[t + 1]]
			var p2: Vector3 = wp[idx[t + 2]]
			t += 3
			# 只保留头发可能触及的区域（头/颈/肩/胸/背），省内存与时间
			if maxf(p0.y, maxf(p1.y, p2.y)) < 1.00: continue
			if minf(p0.y, minf(p1.y, p2.y)) > 1.90: continue
			if maxf(absf(p0.x), maxf(absf(p1.x), absf(p2.x))) > 0.45: continue
			tris.append([p0, p1, p2])
			if is_head:
				tris_head.append(tris.size() - 1)
			else:
				tris_body.append(tris.size() - 1)

func build_grid() -> void:
	for i in tris.size():
		var tr: Array = tris[i]
		var mn: Vector3 = (tr[0] as Vector3).min(tr[1]).min(tr[2])
		var mx: Vector3 = (tr[0] as Vector3).max(tr[1]).max(tr[2])
		var x0 := int(floor(mn.x / cell)); var x1 := int(floor(mx.x / cell))
		var y0 := int(floor(mn.y / cell)); var y1 := int(floor(mx.y / cell))
		var z0 := int(floor(mn.z / cell)); var z1 := int(floor(mx.z / cell))
		for x in range(x0, x1 + 1):
			for y in range(y0, y1 + 1):
				for z in range(z0, z1 + 1):
					var key := x * 1000003 + y * 10007 + z
					if not grid.has(key): grid[key] = []
					(grid[key] as Array).append(i)

func _inside_subset(p: Vector3, subset: Array) -> bool:
	var d := Vector3(1.0, 0.371, 0.193).normalized()
	var cnt := 0
	for i in subset:
		var tr: Array = tris[i]
		var e1: Vector3 = (tr[1] as Vector3) - (tr[0] as Vector3)
		var e2: Vector3 = (tr[2] as Vector3) - (tr[0] as Vector3)
		var h := d.cross(e2)
		var det := e1.dot(h)
		if absf(det) < 1e-12: continue
		var inv := 1.0 / det
		var s: Vector3 = p - (tr[0] as Vector3)
		var u := s.dot(h) * inv
		if u < 0.0 or u > 1.0: continue
		var q := s.cross(e1)
		var vv := d.dot(q) * inv
		if vv < 0.0 or u + vv > 1.0: continue
		if e2.dot(q) * inv > 1e-6: cnt += 1
	return (cnt % 2) == 1

func ray_inside(p: Vector3) -> bool:
	var d := Vector3(1.0, 0.371, 0.193).normalized()
	var cnt := 0
	var cx := int(floor(p.x / cell)); var cy := int(floor(p.y / cell)); var cz := int(floor(p.z / cell))
	var cand := PackedInt32Array()
	for dx in range(-2, 3):
		for dy in range(-2, 3):
			for dz in range(-2, 3):
				var key := (cx + dx) * 1000003 + (cy + dy) * 10007 + (cz + dz)
				if grid.has(key):
					cand.append_array(grid[key])
	for i in cand:
		var tr: Array = tris[i]
		var e1: Vector3 = (tr[1] as Vector3) - (tr[0] as Vector3)
		var e2: Vector3 = (tr[2] as Vector3) - (tr[0] as Vector3)
		var h := d.cross(e2)
		var det := e1.dot(h)
		if absf(det) < 1e-12: continue
		var inv := 1.0 / det
		var s: Vector3 = p - (tr[0] as Vector3)
		var u := s.dot(h) * inv
		if u < 0.0 or u > 1.0: continue
		var q := s.cross(e1)
		var vv := d.dot(q) * inv
		if vv < 0.0 or u + vv > 1.0: continue
		var tt := e2.dot(q) * inv
		if tt > 1e-6: cnt += 1
	return (cnt % 2) == 1

func depth(p: Vector3) -> float:
	# 最近三角形距离（配合 inside 判定得到穿透深度）
	var best := 1e9
	var cx := int(floor(p.x / cell)); var cy := int(floor(p.y / cell)); var cz := int(floor(p.z / cell))
	for dx in range(-3, 4):
		for dy in range(-3, 4):
			for dz in range(-3, 4):
				var key := (cx + dx) * 1000003 + (cy + dy) * 10007 + (cz + dz)
				if not grid.has(key): continue
				for i in (grid[key] as Array):
					var tr: Array = tris[i]
					best = minf(best, _tri_dist(p, tr[0], tr[1], tr[2]))
	return best

func _tri_dist(p: Vector3, a: Vector3, b: Vector3, c: Vector3) -> float:
	var ab := b - a; var ac := c - a; var ap := p - a
	var d1 := ab.dot(ap); var d2 := ac.dot(ap)
	if d1 <= 0.0 and d2 <= 0.0: return ap.length()
	var bp := p - b
	var d3 := ab.dot(bp); var d4 := ac.dot(bp)
	if d3 >= 0.0 and d4 <= d3: return bp.length()
	var vc := d1 * d4 - d3 * d2
	if vc <= 0.0 and d1 >= 0.0 and d3 <= 0.0:
		return (ap - ab * (d1 / (d1 - d3))).length()
	var cp := p - c
	var d5 := ab.dot(cp); var d6 := ac.dot(cp)
	if d6 >= 0.0 and d5 <= d6: return cp.length()
	var vb := d5 * d2 - d1 * d6
	if vb <= 0.0 and d2 >= 0.0 and d6 <= 0.0:
		return (ap - ac * (d2 / (d2 - d6))).length()
	var va := d3 * d6 - d5 * d4
	if va <= 0.0 and (d4 - d3) >= 0.0 and (d5 - d6) >= 0.0:
		return (p - (b + (c - b) * ((d4 - d3) / ((d4 - d3) + (d5 - d6))))).length()
	var den := 1.0 / (va + vb + vc)
	return (p - (a + ab * (vb * den) + ac * (vc * den))).length()

func _init():
	var scene: Node = (load("res://character_creator.tscn") as PackedScene).instantiate()
	get_root().add_child(scene)
	await process_frame
	await process_frame
	for n in scene.find_children("*", "Skeleton3D", true, false):
		if n.get_bone_count() > 200: sk = n
	hp = scene.get_node_or_null("HairPhysics")
	for i in 90: await physics_frame
	for c in sk.get_children():
		if c is MeshInstance3D and c.name in ["head", "body"]:
			build_tris(c, c.name == "head")
	print("ST 三角形数 = ", tris.size())
	build_grid()
	# 头部网格的包围盒（用于报告）
	var worst_in := 0.0
	var inside_total := 0
	var checked := 0
	var free_in := 0
	var free_worst := 0.0
	var pin_in := 0
	var pin_worst := 0.0
	for ci in hp._chains.size():
		var p: PackedVector3Array = hp._chains[ci]["p"]
		var na: int = hp._attach[ci] if ci < hp._attach.size() else 1
		for j in p.size():
			checked += 1
			if ray_inside(p[j]):
				inside_total += 1
				var d := depth(p[j])
				worst_in = maxf(worst_in, d)
				if j < na:
					pin_in += 1
					pin_worst = maxf(pin_worst, d)
				else:
					free_in += 1
					free_worst = maxf(free_worst, d)
	print("ST ★非发根关节在 mesh 内 = %d 个（最深 %.1f mm）   ← 硬要求：必须是 0" % [
		free_in, free_worst * 1000.0])
	print("ST   发根固定段在 mesh 内 = %d 个（最深 %.1f mm） ← 允许（发根本来就长在头皮里）" % [
		pin_in, pin_worst * 1000.0])
	print("ST 链条关节共 %d 个，严格判定在 mesh 内部的有 %d 个，最深 %.1f mm" % [
		checked, inside_total, worst_in * 1000.0])
	# 自检：已知内/外的点
	print("ST 自检 头心(0,1.67,0) inside=", ray_inside(Vector3(0, 1.67, 0)),
		"  脸上方(0,2.20,0) inside=", ray_inside(Vector3(0, 2.20, 0)),
		"  面前(0,1.67,0.25) inside=", ray_inside(Vector3(0, 1.67, 0.25)),
		"  胸内(0,1.35,0) inside=", ray_inside(Vector3(0, 1.35, 0)))
	# 逐链统计
	for ci in hp._chains.size():
		var p: PackedVector3Array = hp._chains[ci]["p"]
		var nin := 0
		var deepest := 0.0
		for j in p.size():
			if ray_inside(p[j]):
				nin += 1
				deepest = maxf(deepest, depth(p[j]))
		if nin > 0:
			print("ST   %s链%d %-6s 内部关节 %d/%d 最深 %.1fmm 根=%s 梢=%s" % [
				"导引" if hp._chains[ci]["guide"] else "插值", ci, hp._chains[ci]["part"],
				nin, p.size(), deepest * 1000.0, str(p[0]), str(p[p.size()-1])])
	# 对比两种判据：椭球 vs 严格网格
	# 两条判据是否一致：严格说"在内部"的关节里，有多少是碰撞场也认为实心的？
	var both := 0
	var only_strict := 0
	for ci in hp._chains.size():
		var p3: PackedVector3Array = hp._chains[ci]["p"]
		for j in p3.size():
			if ray_inside(p3[j]):
				if hp._sdf_solid(p3[j]):
					both += 1
				else:
					only_strict += 1
	print("ST 判据统计：严格在内 %d 个；其中碰撞场也判实心 %d 个，仅严格判内（=碰撞场漏洞）%d 个" % [
		inside_total, both, only_strict])
	var in_head := 0
	var in_body := 0
	for ci in hp._chains.size():
		var p4: PackedVector3Array = hp._chains[ci]["p"]
		for j in p4.size():
			if _inside_subset(p4[j], tris_head):
				in_head += 1
			if _inside_subset(p4[j], tris_body):
				in_body += 1
	print("ST 细分：在头模内 %d 个，在身体内 %d 个（共 %d 关节）" % [in_head, in_body, checked])
	print("ST 判据对比（椭球深度>0=在椭球内；网格内=严格）：")
	for ci in [0, 4]:
		var p2: PackedVector3Array = hp._chains[ci]["p"]
		for j in p2.size():
			print("ST   链%d 关节%2d p=(%.3f,%.3f,%.3f) 椭球=%+.1fmm 网格内=%s 网格深度=%5.1fmm" % [
				ci, j, p2[j].x, p2[j].y, p2[j].z,
				hp._signed_depth(p2[j]) * 1000.0, str(ray_inside(p2[j])), depth(p2[j]) * 1000.0])
	# 头部碰撞体在当前 pose 下的实际位置
	print("ST 头骨 pose=", sk.get_bone_pose_rotation(hp._head), " scale=", sk.get_bone_pose_scale(hp._head))
	print("ST 头骨 global_pose=", sk.get_bone_global_pose(hp._head))
	print("ST 头骨 global_rest=", sk.get_bone_global_rest(hp._head))
	var dn := 0
	for cc in hp._colliders:
		if int(cc["bone"]) != hp._head: continue
		dn += 1
		var pc: Array = hp._posed_collider(cc)
		print("ST   头部椭球%d center=(%.3f,%.3f,%.3f) half=(%.3f,%.3f,%.3f)" % [
			dn, pc[0].x, pc[0].y, pc[0].z, cc["half"].x, cc["half"].y, cc["half"].z])
	# 头模当前 pose 的顶点包围盒
	var hmi: MeshInstance3D = null
	for c in sk.get_children():
		if c is MeshInstance3D and c.name == "head": hmi = c
	if hmi != null:
		var skin2: Skin = hmi.skin
		var mn := Vector3(1e9,1e9,1e9); var mx := Vector3(-1e9,-1e9,-1e9)
		for surf in hmi.mesh.get_surface_count():
			var a: Array = hmi.mesh.surface_get_arrays(surf)
			var v: PackedVector3Array = a[Mesh.ARRAY_VERTEX]
			var bo: PackedInt32Array = a[Mesh.ARRAY_BONES]
			var we: PackedFloat32Array = a[Mesh.ARRAY_WEIGHTS]
			for i in v.size():
				var acc := Vector3.ZERO; var ws := 0.0
				for k in 4:
					var w: float = we[i*4+k]
					if w <= 0.0: continue
					acc += (sk.get_bone_global_pose(bo[i*4+k]) * skin2.get_bind_pose(bo[i*4+k])) * v[i] * w
					ws += w
				if ws > 0.0:
					var q: Vector3 = acc / ws
					mn = mn.min(q); mx = mx.max(q)
		print("ST 头模当前 pose AABB = (%.3f,%.3f,%.3f)..(%.3f,%.3f,%.3f)" % [mn.x,mn.y,mn.z,mx.x,mx.y,mx.z])
	print("ST_END")
	quit()
