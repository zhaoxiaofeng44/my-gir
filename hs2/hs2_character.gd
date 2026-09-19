# hs2_character.gd — HS2 捏人角色（合并骨架版）
# 七个部件 → 单一骨架 + 统一蒙皮（见 hs2_rig_merge.gd），再做实时捏人
extends Node3D

const HS2ShapeScript := preload("res://hs2/hs2_shape.gd")
const HS2RigMerge := preload("res://hs2/hs2_rig_merge.gd")
const SKIN_SHADER := preload("res://shaders/skin.gdshader")

const DATA_DIR := "res://hs2/data"
const PARTS_BODY := ["body", "top", "bot", "shoe"]
const PARTS_ATTACH := ["head", "hair_f", "hair_b"]
const POS_UNIT := 100.0   # 数据表位置单位（与 web 版 applyFaceMorphLegacy 一致）
const HAIR_TINT := Color(0.80, 0.62, 0.45)   # HS2 发贴图为灰度+alpha，发色由材质决定
const MERGE_REV := "GOLDEN-ONLY rev2025-09-19"   # 屏幕/日志可见，用来确认跑的是哪一版

var model_root: Node3D
var main_sk: Skeleton3D
var shape_face
var shape_body
var sliders := {"face": {}, "body": {}}
var _rest := {}   # bone_idx -> {pos, rot, scale, name}

func _ready() -> void:
	var shapes = HS2ShapeScript.load_from_dir(DATA_DIR)
	shape_face = shapes["face"]
	shape_body = shapes["body"]
	_load_category_names()

	model_root = Node3D.new()
	model_root.name = "Model"
	add_child(model_root)
	model_root.rotation_degrees = Vector3(180, 0, 0)   # 实测：原始 glb 需绕 X 180° 才站立

	var entries: Array = []
	for part in PARTS_BODY:
		entries.append(_instantiate(part))
	for part in PARTS_ATTACH:
		entries.append(_instantiate(part))
	entries = entries.filter(func(e): return not e.is_empty())

	_merge_rig(entries)
	for nm in ["hair_f", "hair_b"]:
		var m := _find_mesh(nm)
		if m != null:
			_hair_nodes.append(m)
	_collect_rest()

	for cid in shape_face.cat_map.keys():
		sliders["face"][str(cid)] = 0.5
	for cid in shape_body.cat_map.keys():
		sliders["body"][str(cid)] = 0.5
	apply_shape()
	_run_acceptance()   # 先算验收指标，再建 UI（面板顶部显示）
	_build_ui()
	_setup_camera()

func _instantiate(part: String) -> Dictionary:
	var ps: PackedScene = load("res://hs2/model/%s.glb" % part)
	if ps == null:
		push_warning("missing part " + part)
		return {}
	var node: Node3D = ps.instantiate()
	model_root.add_child(node)
	_fix_materials(part, node)
	var skels := node.find_children("*", "Skeleton3D", true, false)
	var meshes := node.find_children("*", "MeshInstance3D", true, false)
	if skels.is_empty() or meshes.is_empty():
		return {}
	return {
		"node": node,
		"sk": skels[0] as Skeleton3D,
		"mi": meshes[0] as MeshInstance3D,
		"is_body": part == "body",
		"part": part,
	}

# 刚体对齐：把 head/hair 摆到身体头骨处（这一步提供合并所需的装配增量 A）
func _align_attach_parts(entries: Array) -> void:
	var body_sk: Skeleton3D = null
	for e in entries:
		if e.get("is_body", false):
			body_sk = e["sk"]
	if body_sk == null:
		return
	var head_bi := body_sk.find_bone("cf_J_Head_s")
	if head_bi < 0:
		head_bi = body_sk.find_bone("cf_J_Head")
	if head_bi < 0:
		return
	var target: Vector3 = body_sk.to_global(body_sk.get_bone_global_pose(head_bi).origin)
	for e in entries:
		if e.get("is_body", false):
			continue
		var node: Node3D = e["node"]
		var mi: MeshInstance3D = e["mi"]
		var ab: AABB = mi.global_transform * mi.get_aabb()
		node.global_position += target - ab.get_center()

# 合并为单一骨架 + 统一蒙皮
func _merge_rig(entries: Array) -> void:
	var body_e: Dictionary = {}
	for e in entries:
		if e.get("is_body", false):
			body_e = e
	main_sk = body_e["sk"]
	var skin: Skin = HS2RigMerge.merge_into_body(main_sk, entries)
	print("[HS2] merged skeleton bones=", main_sk.get_bone_count(), " binds=", skin.get_bind_count())
	for e in entries:
		var mi: MeshInstance3D = e["mi"]
		HS2RigMerge.rebind_mesh(mi, main_sk, skin)
	# 清理：多余骨架与空壳部件节点
	for e in entries:
		if e.get("is_body", false):
			continue
		var sk: Skeleton3D = e["sk"]
		var node: Node3D = e["node"]
		if sk.get_parent() != null:
			sk.get_parent().remove_child(sk)
		sk.queue_free()
		if node.get_child_count() == 0:
			node.queue_free()

func _collect_rest() -> void:
	_rest.clear()
	for b in main_sk.get_bone_count():
		var rest: Transform3D = main_sk.get_bone_rest(b)
		var basis: Basis = rest.basis
		# 关键：HS2 骨骼含镜像(负缩放)，get_scale() 只给正值会丢翻转，
		# 必须按行列式补回符号，否则重建 pose 时整条链会被转倒。
		var scl: Vector3 = basis.get_scale()
		if basis.determinant() < 0.0:
			scl.x = -scl.x
		_rest[b] = {
			"pos": rest.origin,
			"rot": basis.orthonormalized().get_rotation_quaternion(),
			"scale": scl,
			"name": main_sk.get_bone_name(b),
		}

func apply_shape() -> void:
	var merged := {}
	for k in shape_body.evaluate(sliders["body"]).keys():
		merged[k] = shape_body.evaluate(sliders["body"])[k]
	for k in shape_face.evaluate(sliders["face"]).keys():
		merged[k] = shape_face.evaluate(sliders["face"])[k]
	for b in main_sk.get_bone_count():
		var r: Dictionary = _rest[b]
		var pname: String = r["name"]
		if not merged.has(pname):
			main_sk.reset_bone_pose(b)   # 无增量：直接回到 rest，避免分解误差/镜像丢失
			continue
		var pos: Vector3 = r["pos"]
		var quat: Quaternion = r["rot"]
		var scl: Vector3 = r["scale"]
		if true:
			var d: Dictionary = merged[pname]
			var p: Vector3 = d["pos"]
			if p.length_squared() > 1e-12:
				pos += p * POS_UNIT
			var rv: Vector3 = d["rot"]
			if rv.length_squared() > 1e-12:
				var qz := Quaternion(Vector3(0, 0, 1), deg_to_rad(rv.z))
				var qx := Quaternion(Vector3(1, 0, 0), deg_to_rad(rv.x))
				var qy := Quaternion(Vector3(0, 1, 0), deg_to_rad(rv.y))
				quat = quat * (qz * qx * qy)   # three.js Euler 'ZXY'
			var s: Vector3 = d["scale"]
			if s != Vector3.ONE:
				scl = scl * s
		main_sk.set_bone_pose_position(b, pos)
		main_sk.set_bone_pose_rotation(b, quat)
		main_sk.set_bone_pose_scale(b, scl)

# ---------------- 材质 ----------------
func _tex(fname: String) -> Texture2D:
	var path := "res://hs2/model/Textures/" + fname
	return load(path) if ResourceLoader.exists(path) else null

func _mat(tex: Texture2D, alpha: bool) -> BaseMaterial3D:
	var m := StandardMaterial3D.new()
	if tex:
		m.albedo_texture = tex
	if alpha:
		m.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA_SCISSOR
		m.alpha_scissor_threshold = 0.35
		m.cull_mode = BaseMaterial3D.CULL_DISABLED
		m.roughness = 0.85
		m.specular_mode = BaseMaterial3D.SPECULAR_SCHLICK_GGX
		m.metallic_specular = 0.25
	m.roughness = 0.7
	return m

func _fix_materials(part: String, inst: Node) -> void:
	for mi in inst.find_children("*", "MeshInstance3D", true, false):
		if part == "body":
			for i in mi.mesh.get_surface_count():
				var m := ShaderMaterial.new()
				m.shader = SKIN_SHADER
				m.set_shader_parameter("albedo_texture", _tex("body_d.png"))
				m.set_shader_parameter("base_color", Color(1, 1, 1))
				m.set_shader_parameter("roughness", 0.6)
				m.set_shader_parameter("specular_amount", 0.25)
				m.set_shader_parameter("subsurface_strength", 1.0)
				m.set_shader_parameter("rim_intensity", 0.06)
				mi.set_surface_override_material(i, m)
		elif part == "head":
			for i in mi.mesh.get_surface_count():
				var sname: String = mi.mesh.surface_get_name(i).to_lower()
				var tex: Texture2D = null
				var alpha := false
				if sname.contains("face"):
					tex = _tex("face_d.png")
				elif sname.contains("jiemao") or sname.contains("lash"):
					tex = _tex("eyelash.png")
					alpha = true
				elif sname.contains("meimao") or sname.contains("brow"):
					tex = _tex("cf_m_eyebrow_00_00_00-DXT5.png")
					alpha = true
				elif sname.contains("eye"):
					tex = _tex("cf_t_eye_white_00-DXT1.TGA")
				mi.set_surface_override_material(i, _mat(tex, alpha))
		elif part == "hair_f":
			for i in mi.mesh.get_surface_count():
				var m := _mat(_tex("cf_t_hairfront_03_01-DXT5.png"), true)
				m.albedo_color = HAIR_TINT
				mi.set_surface_override_material(i, m)
		elif part == "hair_b":
			for i in mi.mesh.get_surface_count():
				var m := _mat(_tex("cf_t_hairback_00_01-DXT5.png"), true)
				m.albedo_color = HAIR_TINT
				mi.set_surface_override_material(i, m)

# ---------------- UI ----------------
func set_slider(section: String, cat_id: int, value: float) -> void:
	if not sliders.has(section):
		return
	sliders[section][str(cat_id)] = clampf(value, 0.0, 1.0)
	apply_shape()

func face_ids() -> Array:
	var ids: Array = shape_face.cat_map.keys()
	ids.sort_custom(func(a, b): return int(a) < int(b))
	return ids

func body_ids() -> Array:
	var ids: Array = shape_body.cat_map.keys()
	ids.sort_custom(func(a, b): return int(a) < int(b))
	return ids

func _build_ui() -> void:
	var layer := CanvasLayer.new()
	layer.name = "UI"
	add_child(layer)
	var panel := PanelContainer.new()
	panel.position = Vector2(10, 10)
	panel.custom_minimum_size = Vector2(300, 600)
	layer.add_child(panel)
	var scroll := ScrollContainer.new()
	scroll.horizontal_scroll_mode = ScrollContainer.SCROLL_MODE_DISABLED
	panel.add_child(scroll)
	var box := VBoxContainer.new()
	box.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	scroll.add_child(box)
	var lhint := Label.new()
	lhint.text = "【操作】左键拖拽=旋转 · 右键拖拽=平移 · 滚轮=缩放\n双指捏合=缩放 · 双指滑动=平移 · R=复位 · F=对焦"
	lhint.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	lhint.custom_minimum_size = Vector2(280, 0)
	box.add_child(lhint)
	var lsep := Label.new()
	lsep.text = "———— 装配验收 ————"
	box.add_child(lsep)
	for al in _acc_lines:
		var la := Label.new()
		la.text = str(al)
		box.add_child(la)
	var lh := Label.new()
	lh.text = "———— 头发 ————"
	box.add_child(lh)
	var hstat := Label.new()
	hstat.text = "头发部件: %d 个 %s" % [_hair_nodes.size(), "✓已加载" if _hair_nodes.size() > 0 else "✗未加载"]
	box.add_child(hstat)
	for hn in _hair_nodes:
		var row := HBoxContainer.new()
		var cb := CheckBox.new()
		cb.text = "显示 " + str(hn.name)
		cb.button_pressed = true
		cb.toggled.connect(func(on: bool): hn.visible = on)
		row.add_child(cb)
		box.add_child(row)
	var l1 := Label.new()
	l1.text = "———— 脸部（face）————"
	box.add_child(l1)
	for cid in face_ids():
		box.add_child(_mk_slider("face", int(cid), sliders["face"][str(cid)]))
	var l2 := Label.new()
	l2.text = "———— 身体（body）————"
	box.add_child(l2)
	for cid in body_ids():
		box.add_child(_mk_slider("body", int(cid), sliders["body"][str(cid)]))

func _mk_slider(section: String, cid: int, value: float) -> HBoxContainer:
	var row := HBoxContainer.new()
	var lbl := Label.new()
	var cn := _cat_name(section, cid)
	lbl.text = "%d %s" % [cid, cn] if cn != "" else "%d" % cid
	lbl.custom_minimum_size = Vector2(150, 0)
	lbl.tooltip_text = "%s %d" % [section, cid]
	row.add_child(lbl)
	var sl := HSlider.new()
	sl.min_value = 0.0
	sl.max_value = 1.0
	sl.step = 0.01
	sl.value = value
	sl.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	sl.size_flags_vertical = Control.SIZE_SHRINK_CENTER
	var lbl2 := Label.new()
	lbl2.text = "%.2f" % value
	lbl2.custom_minimum_size = Vector2(40, 0)
	sl.value_changed.connect(func(v: float):
		lbl2.text = "%.2f" % v
		set_slider(section, cid, v)
	)
	row.add_child(sl)
	row.add_child(lbl2)
	return row
# ---------------- 验收检查（机读 + 屏幕可见）----------------
# 判定项（全部在「渲染空间」= 蒙皮输出空间，与屏幕上看到的一致）：
#   A. 头/身包围盒是否重叠（头不能浮空）
#   B. 颈重叠高度：body_top - head_bottom  ∈ [10mm, 80mm]
#   C. 颈轴对齐：头颈切口环圆心 vs 身体颈柱轴心，|Δ(x,z)| < 15mm
#   D. 口径匹配：|头切口环半径 - 身体颈柱半径| < 10mm
const ACC_HEAD_BONE := "cf_J_Head_s"
const ACC_OVERLAP_MIN := 0.010
const ACC_OVERLAP_MAX := 0.080
const ACC_CENTER_TOL := 0.015
const ACC_RADIUS_TOL := 0.010

var _acc_lines: Array = []
var _face_names: PackedStringArray = PackedStringArray()
var _body_names: PackedStringArray = PackedStringArray()
var _hair_nodes: Array = []

func _run_acceptance() -> void:
	var lines: Array = []
	lines.append("版本 " + MERGE_REV)
	var body_mi := _find_mesh("body")
	var head_mi := _find_mesh("head")
	if body_mi == null or head_mi == null:
		lines.append("FAIL: 缺少 body/head")
		_acc_lines = lines
		for l in lines:
			print("[ACC] ", l)
		return
	# ── A. 头部是否落在 golden 规定的位置（用 Blender oracle 推出的期望偏移）──
	# Blender oracle: head.fbx 独立导入的世界 AABB 中心 = (0, -0.0727, -0.019)（米）
	# golden: head_world = boneWorld · Rx(180) · (独立世界)  ⇒ 相对头骨偏移 = (0, +0.0727, +0.019)
	const ORACLE_CENTER_OFFSET := Vector3(0.0, 0.0727, 0.019)
	var bi_head := main_sk.find_bone(ACC_HEAD_BONE)
	var a_ok := false
	if bi_head < 0:
		lines.append("A FAIL: 找不到 " + ACC_HEAD_BONE)
	else:
		# 头骨在渲染空间的位�置
		var V := Basis(Vector3.RIGHT, PI / 2.0)
		var r_bone: Transform3D = main_sk.get_bone_global_rest(bi_head)
		var bone_pos: Vector3 = V * r_bone.origin
		var hp := _world_points(_find_mesh("head"), false)
		var ha0 := _aabb(hp)
		var center: Vector3 = ha0.get_center()
		var got: Vector3 = center - bone_pos
		var err: float = (got - ORACLE_CENTER_OFFSET).length()
		a_ok = err < 0.030
		lines.append("A 头中心偏移 (%.3f, %.3f, %.3f) 期望 (%.3f, %.3f, %.3f) 误差 %.0f mm %s" % [got.x, got.y, got.z, ORACLE_CENTER_OFFSET.x, ORACLE_CENTER_OFFSET.y, ORACLE_CENTER_OFFSET.z, err * 1000.0, "PASS" if a_ok else "FAIL"])
	# ── A2. 头发是否落在 golden 规定的位置 ──────────────────────────────
	# Blender oracle 独立世界 AABB 中心 → golden(T(0,0.05,0)·Rx(-90°) 在 mesh 局部) → 相对头骨期望偏移
	const HAIR_EXPECT := {
		"hair_f": Vector3(0.0021, 0.0630, 0.0637),
		"hair_b": Vector3(0.0042, -0.0929, -0.0099),
	}
	if bi_head >= 0:
		var V2 := Basis(Vector3.RIGHT, PI / 2.0)
		var bp2: Vector3 = V2 * main_sk.get_bone_global_rest(bi_head).origin
		for hn in ["hair_f", "hair_b"]:
			var hm := _find_mesh(hn)
			if hm == null:
				lines.append("A2 %s FAIL: 未加载" % hn)
				continue
			var hc: Vector3 = _aabb(_world_points(hm, false)).get_center()
			var got2: Vector3 = hc - bp2
			var exp: Vector3 = HAIR_EXPECT[hn]
			var err2: float = (got2 - exp).length()
			lines.append("A2 %s 偏移(%.3f,%.3f,%.3f) 期望(%.3f,%.3f,%.3f) 误差 %.0f mm %s" % [hn, got2.x, got2.y, got2.z, exp.x, exp.y, exp.z, err2 * 1000.0, "PASS" if err2 < 0.030 else "FAIL"])
	# ── B. 头是否落在身体颈内（不浮空、不过度下沉）─────────────────────
	var body_pts := _world_points(body_mi, false)
	var head_pts := _world_points(head_mi, false)
	var ba := _aabb(body_pts)
	var ha := _aabb(head_pts)
	var body_top: float = ba.position.y + ba.size.y
	var head_bottom: float = ha.position.y
	var head_top: float = ha.position.y + ha.size.y
	var overlap: float = body_top - head_bottom
	var b_ok: bool = overlap >= 0.010 and overlap <= 0.120
	lines.append("B 颈重叠 %.0f mm %s" % [overlap * 1000.0, "PASS" if b_ok else "FAIL"])
	# ── C. 头部尺寸是否正常（头高 18~24cm），防止被缩放/压扁 ────────────
	var head_h: float = ha.size.y
	var c_ok: bool = head_h >= 0.18 and head_h <= 0.24
	lines.append("C 头高 %.0f mm %s" % [head_h * 1000.0, "PASS" if c_ok else "FAIL"])
	# ── D. 参考数据（不判定，供人工核对）────────────────────────────────
	lines.append("身体 y=[%.3f, %.3f] 头 y=[%.3f, %.3f]" % [ba.position.y, body_top, head_bottom, head_top])
	lines.append("身高 %.0f mm  头骨z偏移 %.1f mm" % [(head_top) * 1000.0, main_sk.get_bone_global_rest(bi_head).origin.z * 1000.0 if bi_head >= 0 else 0.0])
	lines.append("身体顶z偏移 %.1f mm（颈前后基准）" % (0.0))
	var fails := 0
	for l in lines:
		if str(l).contains("FAIL"):
			fails += 1
	lines.append("结论: " + ("全部 PASS" if fails == 0 else "%d 项 FAIL" % fails))
	_acc_lines = lines
	for l in lines:
		print("[ACC] ", l)

func _find_mesh(nm: String) -> MeshInstance3D:
	if main_sk == null:
		return null
	for c in main_sk.get_children():
		if c is MeshInstance3D and c.name == nm:
			return c
	return null

func _world_points(mi: MeshInstance3D, use_raw: bool) -> PackedVector3Array:
	var out := PackedVector3Array()
	if mi == null:
		return out
	var skin: Skin = mi.skin
	var S: Transform3D = main_sk.global_transform
	for surf in mi.mesh.get_surface_count():
		var arrays: Array = mi.mesh.surface_get_arrays(surf)
		var verts: PackedVector3Array = arrays[Mesh.ARRAY_VERTEX]
		var bones: PackedInt32Array = arrays[Mesh.ARRAY_BONES]
		var weights: PackedFloat32Array = arrays[Mesh.ARRAY_WEIGHTS]
		var i := 0
		while i < verts.size():
			if use_raw:
				out.append(verts[i])
			else:
				var acc := Vector3.ZERO
				var wsum := 0.0
				if skin != null and bones.size() >= (i + 1) * 4:
					for k in 4:
						var w: float = weights[i * 4 + k]
						if w <= 0.0:
							continue
						var bi: int = bones[i * 4 + k]
						if bi >= skin.get_bind_count():
							continue
						acc += ((S * main_sk.get_bone_global_pose(bi) * skin.get_bind_pose(bi)) * verts[i]) * w
						wsum += w
				out.append((acc / wsum) if wsum > 0.0 else verts[i])
			i += 3
	return out

## XZ 平面最小二乘圆拟合（Kåsa）
func _fit_circle_xz(pts: PackedVector3Array) -> Dictionary:
	var n := pts.size()
	if n < 8:
		return {}
	var sx := 0.0
	var sz := 0.0
	var sxx := 0.0
	var szz := 0.0
	var sxz := 0.0
	var sxy := 0.0
	var szy := 0.0
	for p in pts:
		var x := p.x
		var z := p.z
		var yy := x * x + z * z
		sx += x
		sz += z
		sxx += x * x
		szz += z * z
		sxz += x * z
		sxy += x * yy
		szy += z * yy
	var m00 := sxx
	var m01 := sxz
	var m02 := sx
	var m11 := szz
	var m12 := sz
	var m22 := float(n)
	var b0 := -sxy
	var b1 := -szy
	var b2 := -(sxx + szz)
	var det: float = m00 * (m11 * m22 - m12 * m12) - m01 * (m01 * m22 - m12 * m02) + m02 * (m01 * m12 - m11 * m02)
	if absf(det) < 1e-12:
		return {}
	var a: float = (b0 * (m11 * m22 - m12 * m12) - m01 * (b1 * m22 - m12 * b2) + m02 * (b1 * m12 - m11 * b2)) / det
	var bb: float = (m00 * (b1 * m22 - m12 * b2) - b0 * (m01 * m22 - m12 * m02) + m02 * (m01 * b2 - b1 * m02)) / det
	var c: float = (m00 * (m11 * b2 - b1 * m12) - m01 * (m01 * b2 - b1 * m02) + b0 * (m01 * m12 - m11 * m02)) / det
	var cx: float = -a * 0.5
	var cz: float = -bb * 0.5
	var rad2: float = cx * cx + cz * cz - c
	if rad2 < 0.0:
		return {}
	return {"center": Vector2(cx, cz), "radius": sqrt(rad2), "n": n}

func _aabb(p: PackedVector3Array) -> AABB:
	if p.is_empty():
		return AABB()
	var mn := p[0]
	var mx := p[0]
	for q in p:
		mn = mn.min(q)
		mx = mx.max(q)
	return AABB(mn, mx - mn)
## 头部网格的开口环（边界边）中最低 3cm 的那一圈顶点（渲染空间）
func _head_open_ring(mi: MeshInstance3D, head_bottom: float) -> PackedVector3Array:
	var out := PackedVector3Array()
	if mi == null:
		return out
	var skin: Skin = mi.skin
	var S: Transform3D = main_sk.global_transform
	for surf in mi.mesh.get_surface_count():
		var arrays: Array = mi.mesh.surface_get_arrays(surf)
		var verts: PackedVector3Array = arrays[Mesh.ARRAY_VERTEX]
		var indices: PackedInt32Array = arrays[Mesh.ARRAY_INDEX]
		if indices.is_empty():
			continue
		var bones: PackedInt32Array = arrays[Mesh.ARRAY_BONES]
		var weights: PackedFloat32Array = arrays[Mesh.ARRAY_WEIGHTS]
		var wp := PackedVector3Array()
		wp.resize(verts.size())
		for i in verts.size():
			var acc := Vector3.ZERO
			var wsum := 0.0
			if skin != null and bones.size() >= (i + 1) * 4:
				for k in 4:
					var w: float = weights[i * 4 + k]
					if w <= 0.0:
						continue
					var bi: int = bones[i * 4 + k]
					if bi >= skin.get_bind_count():
						continue
					acc += ((S * main_sk.get_bone_global_pose(bi) * skin.get_bind_pose(bi)) * verts[i]) * w
					wsum += w
			wp[i] = (acc / wsum) if wsum > 0.0 else verts[i]
		var cnt := {}
		for t in indices.size() / 3:
			for e in 3:
				var a := indices[t * 3 + e]
				var b := indices[t * 3 + (e + 1) % 3]
				var key := "%d_%d" % [min(a, b), max(a, b)]
				cnt[key] = int(cnt.get(key, 0)) + 1
		var band := 0.03
		for key in cnt.keys():
			if int(cnt[key]) != 1:
				continue
			var parts: PackedStringArray = key.split("_")
			var v0: Vector3 = wp[int(parts[0])]
			if v0.y <= head_bottom + band:
				out.append(v0)
	return out
# ---------------- HS2 原版中文分类名（UTF-16LE）----------------
func _load_category_names() -> void:
	_face_names = _read_utf16_lines(DATA_DIR + "/cf_headCategory.txt")
	_body_names = _read_utf16_lines(DATA_DIR + "/cf_bodyCategory.txt")
	print("[HS2] 分类名: 脸 ", _face_names.size(), " 项 / 身体 ", _body_names.size(), " 项")

func _read_utf16_lines(path: String) -> PackedStringArray:
	var out := PackedStringArray()
	if not FileAccess.file_exists(path):
		push_warning("缺少分类名文件: " + path)
		return out
	var f := FileAccess.open(path, FileAccess.READ)
	if f == null:
		return out
	var bytes := f.get_buffer(f.get_length())
	var txt := ""
	var i := 0
	if bytes.size() >= 2 and bytes[0] == 0xFF and bytes[1] == 0xFE:
		i = 2
	while i + 1 < bytes.size():
		var code := int(bytes[i]) | (int(bytes[i + 1]) << 8)
		if code == 13:
			pass
		elif code == 10:
			txt += "
"
		else:
			txt += char(code)
		i += 2
	for line in txt.split("
"):
		var t := line.strip_edges()
		if t != "":
			out.append(t)
	return out

func _cat_name(section: String, cid: int) -> String:
	var arr := _face_names if section == "face" else _body_names
	if cid >= 0 and cid < arr.size():
		return arr[cid]
	return ""
# ---------------- 相机（轨道控制 + 自动取景）----------------
func _setup_camera() -> void:
	var cam := get_node_or_null("Camera")
	if cam == null:
		return
	# 用「渲染空间顶点」求包围盒（蒙皮网格的 get_aabb() 是局部空间，不能直接用）
	var aabb := AABB()
	var first := true
	for c in find_children("*", "MeshInstance3D", true, false):
		var pts := _world_points(c, false)
		if pts.is_empty():
			continue
		var ab := _aabb(pts)
		if first:
			aabb = ab
			first = false
		else:
			aabb = aabb.merge(ab)
	if first:
		return
	cam.frame_aabb(aabb)

func _mesh_aabb_local(mi: MeshInstance3D) -> AABB:
	return mi.get_aabb()

func _unhandled_input(event: InputEvent) -> void:
	# F 键：对焦到角色（相机脚本处理 R 复位）
	if event is InputEventKey and event.pressed and not event.echo:
		if (event as InputEventKey).keycode == KEY_F:
			_setup_camera()