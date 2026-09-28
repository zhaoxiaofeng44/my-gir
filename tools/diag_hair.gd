extends SceneTree
# 头发物理验收：链条几何 / 不可伸缩 / 身体碰撞 / 鸟群分离 / 链条插值 / 形变是否真的作用到网格
var sk: Skeleton3D
var hp: Node
var hf: MeshInstance3D
var hb: MeshInstance3D
var hd: MeshInstance3D
var head := -1
var head_grest := Transform3D()

func pts(mi: MeshInstance3D) -> PackedVector3Array:
	var skin: Skin = mi.skin
	var out := PackedVector3Array()
	for surf in mi.mesh.get_surface_count():
		var a: Array = mi.mesh.surface_get_arrays(surf)
		var v: PackedVector3Array = a[Mesh.ARRAY_VERTEX]
		var bo: PackedInt32Array = a[Mesh.ARRAY_BONES]
		var we: PackedFloat32Array = a[Mesh.ARRAY_WEIGHTS]
		for i in v.size():
			var acc := Vector3.ZERO
			var ws := 0.0
			for k in 4:
				var w: float = we[i * 4 + k]
				if w <= 0.0: continue
				var bi: int = bo[i * 4 + k]
				acc += (sk.get_bone_global_pose(bi) * skin.get_bind_pose(bi)) * v[i] * w
				ws += w
			out.append(acc / ws if ws > 0.0 else v[i])
	return out

func centroid(p: PackedVector3Array) -> Vector3:
	var c := Vector3.ZERO
	for q in p: c += q
	return c / float(maxi(p.size(), 1))

func maxdiff(a: PackedVector3Array, b: PackedVector3Array) -> float:
	var m := 0.0
	for i in mini(a.size(), b.size()):
		m = maxf(m, a[i].distance_to(b[i]))
	return m

func settle(n: int) -> void:
	for i in n:
		await physics_frame

func _init():
	var scene: Node = (load("res://character_creator.tscn") as PackedScene).instantiate()
	get_root().add_child(scene)
	await process_frame
	await process_frame
	for n in scene.find_children("*", "Skeleton3D", true, false):
		if n.get_bone_count() > 200: sk = n
	hp = scene.get_node_or_null("HairPhysics")
	for c in sk.get_children():
		if c is MeshInstance3D:
			if c.name == "hair_f": hf = c
			elif c.name == "hair_b": hb = c
			elif c.name == "head": hd = c
	head = sk.find_bone("cf_J_Head_s")
	head_grest = sk.get_bone_global_rest(head)
	print("HF 发链=", hp.chain_count(), " 导引=", hp.guide_count(), " 骨=", sk.get_bone_count())
	print("HF 参数 刚度=%.2f 阻尼=%.2f 重力=%.1f 分离=%.3f 关节/链=%d" % [
		hp.stiffness, hp.drag, hp.gravity, hp.boid_separation, hp.joints_per_chain])

	# 链几何：打印每条导引链的根/梢，确认链确实躺在头发上
	var guides: Array = hp._guides
	for gi in guides:
		var ch: Dictionary = hp._chains[gi]
		var q: PackedVector3Array = ch["q"]
		print("  链%d %s 根=(%.3f,%.3f,%.3f) 梢=(%.3f,%.3f,%.3f) 长=%.3fm" % [
			gi, ch["part"], q[0].x, q[0].y, q[0].z, q[q.size()-1].x, q[q.size()-1].y, q[q.size()-1].z, ch["total"]])

	# 先关物理取「真·静止几何」作为基准（否则物理帧可能已经跑过几帧）
	hp.set_enabled(false)
	await settle(3)
	var rest_f := pts(hf)
	var rest_b := pts(hb)
	var rest_h := pts(hd)
	hp.set_enabled(true)
	hp.reset_physics()
	await settle(2)
	print("HF 静止AABB hair_f=", _aabb(rest_f), " hair_b=", _aabb(rest_b))
	print("HF 权重归属 hair_f=", hp.weight_usage(hf), " hair_b=", hp.weight_usage(hb))
	for gi in guides:
		var rr: PackedFloat32Array = hp._radius[gi]
		var s := ""
		for v in rr:
			s += "%.0f " % (v * 1000.0)
		print("  链%d 关节厚度(mm): %s" % [gi, s])

	await settle(180)
	hp.measure()
	# 用 measure 里的 stretch 之外，再直接量一遍
	print("0) 静止贴合度: 平均离体=%.1fmm 最大=%.1fmm" % [hp.gap_stats()["mean"] * 1000.0, hp.gap_stats()["max"] * 1000.0])
	print("1) 稳定180帧: 拉伸=%.4f%% 链最小间距=%.1fmm 网格穿透=%.2fmm/%d顶点 位置=%s" % [
		hp.metric_max_stretch * 100.0, hp.metric_min_sep * 1000.0,
		hp.metric_max_pen * 1000.0, hp.metric_pen_verts, str(hp.metric_pen_at)])
	print("1) 稳定贴合度: 平均离体=%.1fmm 最大=%.1fmm" % [hp.gap_stats()["mean"] * 1000.0, hp.gap_stats()["max"] * 1000.0])
	print("1) 静止漂移 hair_f=%.1fmm hair_b=%.1fmm  网格最大形变 %.1f/%.1fmm" % [
		centroid(pts(hf)).distance_to(centroid(rest_f)) * 1000.0,
		centroid(pts(hb)).distance_to(centroid(rest_b)) * 1000.0,
		maxdiff(pts(hf), rest_f) * 1000.0, maxdiff(pts(hb), rest_b) * 1000.0])

	# 2) 绕「头模中心」转头 45°（头骨原点不在头里，必须绕真实头部中心转）
	var aabb: AABB = (head_grest * hd.skin.get_bind_pose(head)) * hd.mesh.get_aabb()
	var pivot: Vector3 = aabb.get_center()
	var R := Basis(Vector3.UP, deg_to_rad(45.0))
	var G := Transform3D(R, pivot - R * pivot) * head_grest
	sk.set_bone_global_pose_override(head, G, 1.0, true)
	await settle(4)
	var lag := maxdiff(pts(hf), rest_f)
	print("2) 绕头中心转45° 4帧: 头发最大位移=%.1fmm（应有惯性滞后、非爆散）" % (lag * 1000.0))
	await settle(200)
	hp.measure()
	var set_f := pts(hf)
	var set_h := pts(hd)
	print("2) 转头稳定: 拉伸=%.4f%% 穿透=%.2fmm/%d顶点 骨=%s 头发位移=%.1fmm 头模位移=%.1fmm" % [
		hp.metric_max_stretch * 100.0, hp.metric_max_pen * 1000.0, hp.metric_pen_verts, hp.metric_pen_bone,
		maxdiff(set_f, rest_f) * 1000.0, maxdiff(set_h, rest_h) * 1000.0])
	# 转头结束后，检查有没有链条关节本身进了身体
	var bad := []
	for r in hp.chain_depth_report():
		if r["inside"] > 0:
			bad.append(r)
	print("2b) 转头后「体内关节数>0」的链 = %d 条" % bad.size())
	for r in bad.slice(0, 6):
		print("2b)   %s链%d 体内最深=%+.1fmm(关节%d) 个数=%d" % [
			"导引" if r["guide"] else "插值", r["ci"], r["any"] * 1000.0, r["any_j"], r["inside"]])
	sk.clear_bones_global_pose_override()
	hp.reset_physics()
	await settle(200)

	# 3) 风 → 偏转 → 停风回位
	var w0 := pts(hb)
	hp.wind = Vector3(25.0, 0.0, 0.0)
	await settle(150)
	var w1 := pts(hb)
	hp.measure()
	print("3) 风25: hair_b 质心偏移=%.1fmm 最大形变=%.1fmm 拉伸=%.4f%% 网格穿透=%.2fmm/%d顶点 位置=%s 链间距=%.1fmm" % [
		centroid(w1).distance_to(centroid(w0)) * 1000.0, maxdiff(w1, w0) * 1000.0,
		hp.metric_max_stretch * 100.0, hp.metric_max_pen * 1000.0, hp.metric_pen_verts,
		str(hp.metric_pen_at), hp.metric_min_sep * 1000.0])
	hp.wind = Vector3.ZERO
	await settle(300)
	print("3) 风停300帧: 残余形变=%.1fmm" % (maxdiff(pts(hb), w0) * 1000.0))

	# 4) 关物理 → 回到静止几何
	hp.set_enabled(false)
	await settle(5)
	sk.clear_bones_global_pose_override()
	await settle(5)
	print("4) 关物理: 与静止几何最大差=%.3fmm（应≈0）" % (maxdiff(pts(hf), rest_f) * 1000.0))

	# 5) 捏人滑杆拉到极端：身体变形后头发是否还穿模
	hp.set_enabled(true)
	hp.reset_physics()
	for cid in [9, 12, 14, 15, 20]:
		scene.set_slider("body", cid, 1.0)
	for cid in [0, 1, 3]:
		scene.set_slider("face", cid, 1.0)
	await settle(120)
	hp.measure()
	print("5) 滑杆拉满(头/肩/胸/腰): 网格穿透=%.2fmm/%d顶点 位置=%s" % [
		hp.metric_max_pen * 1000.0, hp.metric_pen_verts, str(hp.metric_pen_at) + " 骨=" + hp.metric_pen_bone])
	var a0 := pts(hb)
	hp.wind = Vector3(25, 0, 0)
	await settle(120)
	hp.measure()
	print("5) 滑杆拉满 + 风25: 网格穿透=%.2fmm/%d顶点 位置=%s" % [
		hp.metric_max_pen * 1000.0, hp.metric_pen_verts, str(hp.metric_pen_at) + " 骨=" + hp.metric_pen_bone])
	hp.wind = Vector3.ZERO
	print("HF_END")
	quit()

func _aabb(p: PackedVector3Array) -> String:
	if p.is_empty(): return "empty"
	var mn := p[0]; var mx := p[0]
	for q in p:
		mn = mn.min(q); mx = mx.max(q)
	return "(%.3f,%.3f,%.3f)..(%.3f,%.3f,%.3f)" % [mn.x, mn.y, mn.z, mx.x, mx.y, mx.z]
