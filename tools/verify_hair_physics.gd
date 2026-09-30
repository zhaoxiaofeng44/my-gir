# tools/verify_hair_physics.gd — 捏人场景「物理头发 + 布料」独立自检(headless 可跑)
#
# 头发与布料**全部**由 GDExtension(addons/cloth_native)实现, 与
# scenes/cloth_hair_test_suite.tscn / scenes/simple_hair_demo.tscn 是同一套算法。
# 原 hs2/hair_system.gd(HS2 自实现的 GDScript 发链物理)已删除。
#
# 判据(全部机读)：
#   A. 头皮发根贴在头模表面：发根 -> 最近头模顶点的距离(平均/最大)
#   B. 物理响应：静置 vs 甩头，发尖最大位移、粒子最大速度
#   C. 生长方向以后向为主(前向=糊脸)
#   D. 布料挂在腰上：顶行高度 / 裙摆高度 / 钉住点数 / 穿透
#   E. 性能
#
# 用法: godot --headless --path . -s tools/verify_hair_physics.gd
extends SceneTree

func _init() -> void:
	var ps: PackedScene = load("res://character_creator.tscn")
	var ch = ps.instantiate()
	get_root().add_child(ch)
	# 等角色装配完成(骨架 / 物理头发 / 布料都在 _ready 里建)
	for i in 60:
		await process_frame
	var out := {}

	var hn = ch._hair_native
	out["hair_ready"] = hn != null
	out["hair_enabled"] = ch.hair_enabled
	if hn == null:
		print("VERIFY_HAIR ", JSON.stringify(out))
		quit()
		return
	out["guides"] = hn.stats().get("guides", 0)
	out["render_strands"] = hn.stats().get("render", 0)
	out["scalp_roots"] = hn.native_node().get_scalp_count()

	# ---- 静置一段, 让头发垂下来 ----
	_settle(ch, 150)

	# ---- A. 发根 -> 头模表面 ----
	var wr: PackedVector3Array = hn.world_roots()
	var pts: PackedVector3Array = ch._world_points(ch._find_mesh("head"), false)
	var gap_sum := 0.0
	var gap_max := 0.0
	for r in wr:
		var best := 1e9
		for p in pts:
			best = minf(best, r.distance_to(p))
		gap_sum += best
		gap_max = maxf(gap_max, best)
	out["scalp_gap_avg_mm"] = _mm(gap_sum / float(maxi(wr.size(), 1)))
	out["scalp_gap_max_mm"] = _mm(gap_max)

	var pos_a: PackedVector3Array = hn.native_node().get_positions()
	var tips_a := _tips(hn, pos_a)

	# ---- C. 生长方向 ----
	var seg: int = hn.segment_count + 1
	var fwd := 0
	var bwd := 0
	var i := 0
	while i + seg - 1 < pos_a.size():
		var dz: float = pos_a[i + seg - 1].z - pos_a[i].z
		if dz > 0.01:
			fwd += 1
		elif dz < -0.01:
			bwd += 1
		i += seg
	out["strands_front"] = fwd
	out["strands_back"] = bwd
	out["particle_span_m"] = _round(_span(pos_a))

	# ---- B. 甩头：转颈骨, 看发丝是否被甩动 ----
	ch.set_shaking(true)
	_settle(ch, 45)
	var pos_b: PackedVector3Array = hn.native_node().get_positions()
	var tips_b := _tips(hn, pos_b)
	var dmax := 0.0
	for k in mini(tips_a.size(), tips_b.size()):
		dmax = maxf(dmax, tips_a[k].distance_to(tips_b[k]))
	out["shake_tip_move_mm"] = _mm(dmax)
	ch.set_shaking(false)

	# ---- D. 布料 ----
	var cloth = ch._cloth
	out["cloth_ready"] = cloth != null
	if cloth != null:
		_settle(ch, 30)
		var cp: PackedVector3Array = cloth.get_positions()
		var top_y := -1e9
		var bottom_y := 1e9
		var cx := 0.0
		var cz := 0.0
		for p in cp:
			top_y = maxf(top_y, p.y)
			bottom_y = minf(bottom_y, p.y)
			cx += p.x
			cz += p.z
		out["cloth_rows_cols"] = [cloth.rows, cloth.cols]
		out["cloth_top_y"] = _round(top_y)
		out["cloth_bottom_y"] = _round(bottom_y)
		out["cloth_center_xz"] = [_round(cx / float(maxi(cp.size(), 1))), _round(cz / float(maxi(cp.size(), 1)))]
		var pins: PackedByteArray = cloth.get_pinned_flags()
		var pc := 0
		for b in pins:
			pc += b
		out["cloth_pinned"] = pc
		out["cloth_penetration"] = cloth.penetration_count()

	# ---- E. 性能 ----
	_settle(ch, 60)
	out["fps_headless"] = Engine.get_frames_per_second()
	print("VERIFY_HAIR ", JSON.stringify(out))
	quit()

func _settle(ch: Node3D, frames: int) -> void:
	for i in frames:
		ch._process(1.0 / 60.0)
		await process_frame

func _tips(hn, pos: PackedVector3Array) -> PackedVector3Array:
	var out := PackedVector3Array()
	var seg: int = hn.segment_count + 1
	var i := seg - 1
	while i < pos.size():
		out.append(pos[i])
		i += seg
	return out

## 粒子点云包围盒对角线(越大说明头发铺得越开; 挤成一条会接近 0)
func _span(pos: PackedVector3Array) -> float:
	if pos.size() == 0:
		return 0.0
	var mn := pos[0]
	var mx := pos[0]
	for p in pos:
		mn = mn.min(p)
		mx = mx.max(p)
	return (mx - mn).length()

func _mm(v: float) -> float:
	return round(v * 10000.0) / 10.0   # 米 -> 毫米, 保留 1 位

func _round(v: float) -> float:
	return round(v * 1000.0) / 1000.0
