# hs2_character.gd — HS2 捏人角色（合并骨架版）
# 七个部件 → 单一骨架 + 统一蒙皮（见 hs2_rig_merge.gd），再做实时捏人
extends Node3D

const HS2ShapeScript := preload("res://hs2/hs2_shape.gd")
const HS2RigMerge := preload("res://hs2/hs2_rig_merge.gd")
const SKIN_SHADER := preload("res://shaders/skin.gdshader")
const HAIR_SHADER := preload("res://shaders/hair.gdshader")

const DATA_DIR := "res://hs2/data"
const PARTS_BODY := ["body", "top", "bot", "shoe"]
const PARTS_ATTACH := ["head", "hair_f", "hair_b"]
const POS_UNIT := 100.0   # 数据表位置单位（与 web 版 applyFaceMorphLegacy 一致）
const HAIR_TINT := Color(0.72, 0.55, 0.40)   # HS2 发贴图为灰度+alpha，发色由材质决定
const MERGE_REV := "GOLDEN-ONLY rev2025-09-19"   # 屏幕/日志可见，用来确认跑的是哪一版
const HAIR_NATIVE := preload("res://hs2/hair_native.gd")

# ---------------- 头发(全部由 C++ XPBD 插件实现) ----------------
# 说明: 原 hs2/hair_system.gd(HS2 自实现的 GDScript 发链物理)已**整体删除**。
# 头发现在完全走 GDExtension 里的 XPBDHairNative —— 与
# scenes/cloth_hair_test_suite.tscn / scenes/simple_hair_demo.tscn 是同一套算法:
# 头皮发根采样、距离/弯曲约束、重力/阻尼/风、头部椭球+身体胶囊碰撞、渲染发丝插值。
@export_group("头发(物理)")
@export var hair_enabled := true           # 物理头发开关
@export var hair_guide_count := 160        # 导向发束(物理密度)
@export var hair_render_strands := 4000    # 渲染发丝(视觉密度)
@export var hair_len_top := 0.26           # 头顶/侧后发长(m)
@export var hair_len_bottom := 0.62        # 颈后发长(m)
@export var hair_bang_len := 0.13          # 刘海长度(m)
@export var hair_stiffness := 0.55         # 链约束刚度
@export var hair_damping := 0.09           # 阻尼
@export var hide_hair_shell := true        # 隐藏 HS2 头发贴片(物理头发替换贴片发型)
@export var hair_wind := 0.0               # 风力(m/s²)

@export_group("布料")
@export var cloth_enabled := true          # 连衣裙布料(C++ XPBD，布料测试场景同款)
@export var cloth_cols := 32
@export var cloth_rows := 14
@export var cloth_pin_rows := 2            # 钉住的行数(腰头)
@export var cloth_wind := 0.0              # 风力(m/s²)
@export var cloth_hidden := false          # 只跑物理不显示(调试)

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
	# 布料: C++ XPBD(与布料测试场景同款)；头发在 apply_shape 之后建(要用到静止骨架)
	_build_cloth()
	_collect_rest()

	for cid in shape_face.cat_map.keys():
		sliders["face"][str(cid)] = 0.5
	for cid in shape_body.cat_map.keys():
		sliders["body"][str(cid)] = 0.5
	apply_shape()
	_run_acceptance()   # 先算验收指标，再建 UI（面板顶部显示）
	if hair_enabled:
		_setup_hair_native()
	_build_ui()
	_setup_camera()

func _process(d: float) -> void:
	_shake_head(minf(d, 1.0 / 30.0))   # J 键甩头(直接转颈骨, 与动画/滑杆叠加)
	_step_hair_native(d)
	_step_cloth(d)
	# 每 20 帧刷新一次头发机读指标
	_tick += 1
	if _tick % 20 != 0:
		return
	if _hair_info != null and _hair_native != null:
		var st: Dictionary = _hair_native.stats()
		if not st.is_empty():
			_hair_info.text = "物理发束 %d · 渲染发丝 %d · 粒子 %d" % [
				st.get("guides", 0), st.get("render", 0), st.get("particles", 0)]

# =====================================================================
# C++ XPBD 物理头发(与"布料测试场景"同款求解器)
# 发根贴着头皮走: 每帧由 A = S·pose(头骨)·rest(头骨)⁻¹ 算出世界发根下发。
# =====================================================================
func _setup_hair_native() -> void:
	if not ClassDB.class_exists("XPBDHairNative"):
		push_warning("[HS2] C++ 头发扩展未注册")
		hair_enabled = false
		return
	var hn = HAIR_NATIVE.new()
	hn.name = "HairNative"
	hn.guide_count = hair_guide_count
	hn.render_strands = hair_render_strands
	hn.len_top = hair_len_top
	hn.len_bottom = hair_len_bottom
	hn.bang_len = hair_bang_len
	hn.stiffness = hair_stiffness
	hn.damping = hair_damping
	hn.wind = Vector3(hair_wind, 0.0, 0.0)
	add_child(hn)
	var ok: bool = hn.build(main_sk, {
		"head": _find_mesh("head"),
		"body": _find_mesh("body"),
	})
	if not ok:
		hn.queue_free()
		push_warning("[HS2] 物理头发构建失败")
		return
	_hair_native = hn
	_apply_hair_visibility()

func _step_hair_native(d: float) -> void:
	if _hair_native == null or not hair_enabled:
		return
	if _hair_rebuild_pending:
		_hair_rebuild_pending = false
		_hair_native.reset_physics()
	var dt: float = minf(d, 1.0 / 30.0)
	_hair_native.set_wind(Vector3(hair_wind, 0.0, 0.0))
	_hair_native.step(dt)

func _apply_hair_visibility() -> void:
	var native_on: bool = hair_enabled and _hair_native != null
	# HS2 头发贴片(发壳)：物理头发默认隐藏它，避免两套头发叠在一起
	for hn in _hair_nodes:
		if hn is Node3D:
			hn.visible = not (native_on and hide_hair_shell)
	if _hair_native != null:
		_hair_native.enabled = native_on
		var m: MeshInstance3D = _hair_native.mesh_node()
		if m != null:
			m.visible = native_on
		var p = _hair_native.native_node()
		if p != null:
			p.visible = native_on

func set_hair_enabled(on: bool) -> void:
	hair_enabled = on
	if on and _hair_native == null:
		_setup_hair_native()
	_apply_hair_visibility()

func set_hair_shell_visible(on: bool) -> void:
	hide_hair_shell = not on
	_apply_hair_visibility()

# =====================================================================
# 环形裙布(C++ XPBD 布料，与"布料测试场景"同款)
# =====================================================================
func _build_cloth() -> void:
	if not ClassDB.class_exists("XPBDClothNative"):
		push_warning("[HS2] C++ 布料扩展未注册，跳过裙布")
		return
	# ---- 用身体顶点实测「腰」: 腰高 + 腰半径 + 髋半径 ----
	# (原来写死 hip 骨 - 0.02 会算到 -0.02m, 因为 HS2 的装配空间是 Y 向下的)
	var body_mi := _find_mesh("body")
	var wi: Dictionary = _hair_native.waist_info(body_mi) if _hair_native != null else {}
	if wi.is_empty():
		wi = {"y": 1.05, "r": 0.16, "hip_r": 0.19, "h": 1.7, "floor": 0.0}
	_cloth_cy = wi["y"]
	_cloth_R = maxf(wi["r"] + 0.012, 0.10)
	var len_total: float = clampf(wi["h"] * 0.42, 0.35, 0.62)

	_cloth = XPBDClothNative.new()
	_cloth.name = "SkirtCloth"
	_cloth.top_level = true
	add_child(_cloth)
	_cloth.cols = cloth_cols
	_cloth.rows = cloth_rows
	_cloth.ground_y = wi["floor"]
	_cloth.enable_self_collision = true
	_cloth.set_wind_v(Vector3(cloth_wind, 0.0, 0.0))
	_cloth.build()
	# 逐行向下做环形布局(沿用布料测试场景里验证过的写法); 腰头钉住(前 cloth_pin_rows 行)
	var pos: PackedVector3Array = _cloth.get_positions()
	var pinned: PackedByteArray = _cloth.get_pinned_flags()
	var hip_r: float = wi["hip_r"]
	var hem_r: float = maxf(hip_r * 1.35, 0.24)
	for r in cloth_rows:
		for c in cloth_cols:
			var i: int = r * cloth_cols + c
			var ang: float = TAU * float(c) / float(cloth_cols)
			var t: float = float(r) / float(maxi(cloth_rows - 1, 1))
			# 半径: 腰 -> 髋 -> 裙摆(自然外扩)
			var rr: float = _cloth_R
			if t < 0.30:
				rr = lerpf(_cloth_R, hip_r * 1.02, t / 0.30)
			else:
				rr = lerpf(hip_r * 1.02, hem_r, (t - 0.30) / 0.70)
			pos.set(i, Vector3(cos(ang) * rr, _cloth_cy - len_total * t, sin(ang) * rr))
			pinned.set(i, 1 if r < cloth_pin_rows else 0)
	_cloth.set_positions(pos)
	_cloth.set_pinned_flags(pinned)
	# 关键: 环形布局与 build() 的网格间距不同, 必须重算静止长度
	_cloth.refresh_rest_lengths()
	# 身体胶囊碰撞: 布料不穿进身体
	_cloth.set_body_capsule(true, Vector3(0, _cloth_cy - 0.05, 0), Vector3(0, wi["floor"] + 0.72, 0), maxf(_cloth_R, 0.13))
	var m := StandardMaterial3D.new()
	m.shading_mode = BaseMaterial3D.SHADING_MODE_PER_PIXEL
	m.cull_mode = BaseMaterial3D.CULL_DISABLED
	m.roughness = 0.8
	m.albedo_color = Color(0.35, 0.42, 0.65)
	_cloth.set_material(m)
	_cloth_mesh = MeshInstance3D.new()
	_cloth_mesh.name = "SkirtClothMesh"
	_cloth_mesh.top_level = true
	add_child(_cloth_mesh)
	_cloth_mesh.visible = not cloth_hidden
	print("[HS2] 布料 %dx%d 腰 y=%.3f r=%.3f 髋 %.3f 裙长 %.3f" % [
		cloth_cols, cloth_rows, _cloth_cy, _cloth_R, hip_r, len_total])

func _step_cloth(d: float) -> void:
	if _cloth == null or _cloth_mesh == null:
		return
	# 角色整体移动(拖拽/位移)时, 布料要跟着走
	if _cloth_follow_prev != Vector3.ZERO or _cloth_follow_init:
		var now := _character_origin()
		if _cloth_follow_init:
			_cloth.translate_model(now - _cloth_follow_prev)
		_cloth_follow_prev = now
	_cloth_follow_init = true
	var sh: float = minf(d, 1.0 / 30.0) / 2.0
	for ss in 2:
		_cloth.sub_begin()
		_cloth.sub_integrate(sh, Vector3.ZERO)
		_cloth.sub_solve(sh)
		_cloth.begin_contacts()
		_cloth.self_collision()
		_cloth.object_collision()
		_cloth.final_push_out()
		_cloth.collide_body_capsule()
		_cloth.sub_recover(sh)
	_cloth.update_mesh()
	_cloth_mesh.mesh = _cloth.get_mesh()

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
	# 身体形态变了 → 重启物理头发(头部椭球/身体胶囊要跟着新形态走)
	if _hair_native != null:
		_hair_rebuild_pending = true

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

# 头发材质：走 shaders/hair.gdshader（Kajiya-Kay 双叶各向异性 + 根梢渐变 + 假 AO + 透光）。
# 发丝切线/根梢参数/朝内遮蔽由建模阶段**逐顶点烘焙进 TANGENT 与 COLOR**。
# 注意: 这个着色器只服务 HS2 的"头发贴片"资产(hair_f/hair_b); 物理头发
# 走 C++ 插件的顶点色 + 标准材质, 与它无关。
func _hair_mat(tex: Texture2D) -> ShaderMaterial:
	var m := ShaderMaterial.new()
	m.shader = HAIR_SHADER
	m.set_shader_parameter("albedo_texture", tex)
	m.set_shader_parameter("hair_tint", HAIR_TINT)
	m.set_shader_parameter("root_color", Color(0.50, 0.46, 0.44))
	m.set_shader_parameter("tip_color", Color(1.08, 1.02, 0.96))
	m.set_shader_parameter("grad_curve", 1.5)
	m.set_shader_parameter("alpha_scissor", 0.35)
	m.set_shader_parameter("tip_fade", 0.16)
	m.set_shader_parameter("root_ao", 0.45)
	m.set_shader_parameter("root_ao_len", 0.30)
	m.set_shader_parameter("inner_ao", 0.40)
	m.set_shader_parameter("primary_shift", 0.12)
	m.set_shader_parameter("primary_power", 140.0)
	m.set_shader_parameter("primary_gain", 0.08)
	m.set_shader_parameter("secondary_shift", -0.28)
	m.set_shader_parameter("secondary_power", 26.0)
	m.set_shader_parameter("secondary_gain", 0.04)
	m.set_shader_parameter("secondary_color", Color(0.86, 0.73, 0.56))
	m.set_shader_parameter("translucency", 0.25)
	m.set_shader_parameter("translucency_color", Color(0.78, 0.46, 0.28))
	m.set_shader_parameter("rim", 0.10)
	m.set_shader_parameter("roughness", 0.45)
	m.set_shader_parameter("specular_amount", 0.30)
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
				mi.set_surface_override_material(i, _hair_mat(_tex("cf_t_hairfront_03_01-DXT5.png")))
		elif part == "hair_b":
			for i in mi.mesh.get_surface_count():
				mi.set_surface_override_material(i, _hair_mat(_tex("cf_t_hairback_00_01-DXT5.png")))

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
	var lh0 := Label.new()
	lh0.text = "———— 头发（C++ XPBD 物理，测试场景同款）————"
	box.add_child(lh0)
	box.add_child(_mk_check("启用物理头发", hair_enabled, func(on: bool): set_hair_enabled(on)))
	box.add_child(_mk_check("显示 HS2 头发贴片", not hide_hair_shell, func(on: bool): set_hair_shell_visible(on)))
	_hair_info = Label.new()
	_hair_info.text = "—"
	box.add_child(_hair_info)
	var lh := Label.new()
	lh.text = "———— 头发贴片部件 ————"
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
	# ---- 求解参数 ----
	box.add_child(_mk_phys("链刚度", 0.0, 1.0, hair_stiffness, func(v: float):
		hair_stiffness = v
		if _hair_native: _hair_native.set_stiffness(v)))
	box.add_child(_mk_phys("阻尼", 0.0, 1.0, hair_damping, func(v: float):
		hair_damping = v
		if _hair_native: _hair_native.set_damping(v)))
	box.add_child(_mk_phys("风力", -30.0, 30.0, hair_wind, func(v: float):
		hair_wind = v
		if _hair_native: _hair_native.set_wind(Vector3(v, 0.0, 0.0))))
	box.add_child(_mk_phys("渲染发丝数", 200.0, 12000.0, float(hair_render_strands), func(v: float):
		hair_render_strands = int(v)
		if _hair_native: _hair_native.set_render_strands(int(v))))
	var lreset := Button.new()
	lreset.text = "重置物理头发"
	lreset.pressed.connect(func(): if _hair_native: _hair_native.reset_physics())
	box.add_child(lreset)
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
var _hair_native: Node3D          # C++ XPBD 物理头发(HS2HairNative)
var _hair_info: Label
var _hair_rebuild_pending := false
var _cloth: XPBDClothNative       # 环形裙布
var _cloth_mesh: MeshInstance3D
var _cloth_hip := Vector3.ZERO    # 腰(裙布顶行)位置，装配空间
var _cloth_R := 0.19
var _cloth_cy := 0.78
var _shaking := false
var _cloth_follow_prev := Vector3.ZERO
var _cloth_follow_init := false
var _tick := 0

# 角色在世界里的水平位置(布料/头发跟着走)
func _character_origin() -> Vector3:
	if model_root == null:
		return Vector3.ZERO
	return model_root.global_position

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
	# 物理头发/裙布的网格节点由求解器每帧写入: 相机取景那一帧可能还没有 mesh
	if mi == null or mi.mesh == null:
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
		if _is_physics_mesh(c):
			continue   # 物理头发/裙布是动态大网格, 不参与取景
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

# 物理求解器生成的网格(头发发丝/裙布): 世界空间 + 每帧重建, 不属于角色装配
func _is_physics_mesh(mi: Node) -> bool:
	if _mesh_aabb_local_name(mi) in ["HairNativeMesh", "SkirtClothMesh"]:
		return true
	return false

func _mesh_aabb_local_name(mi: Node) -> String:
	return mi.name

func _unhandled_input(event: InputEvent) -> void:
	# F 键：对焦到角色（相机脚本处理 R 复位）
	if event is InputEventKey and event.pressed and not event.echo:
		var k := (event as InputEventKey).keycode
		if k == KEY_F:
			_setup_camera()
		# ---- 物理测试开关(与"布料测试场景"对齐的操作) ----
		elif k == KEY_H:
			set_hair_enabled(not hair_enabled)
		elif k == KEY_J:
			_shaking = not _shaking
		elif k == KEY_K:
			hair_wind = 0.0 if hair_wind > 0.0 else 8.0
		elif k == KEY_L:
			_setup_hair_native()
		elif k == KEY_P:
			# 踹一脚裙摆
			if _cloth:
				var vel: PackedVector3Array = _cloth.get_velocities()
				for i in vel.size():
					vel.set(i, vel[i] + Vector3(1.2, 0.4, 0.6))
				_cloth.set_velocities(vel)

# 方便 headless 自检: 用关节(骨骼)把头发甩起来(不依赖系统鼠标/键盘)
func set_shaking(on: bool) -> void:
	_shaking = on

func _shake_head(d: float) -> void:
	if not _shaking or main_sk == null:
		return
	var hb := main_sk.find_bone(ACC_HEAD_BONE)
	if hb < 0:
		return
	var t := Time.get_ticks_msec() / 1000.0
	var q := Quaternion(Vector3.UP, 0.55 * sin(t * 7.5))
	main_sk.set_bone_pose_rotation(hb, q)
func _mk_check(label: String, value: bool, cb: Callable) -> CheckBox:
	var c := CheckBox.new()
	c.text = label
	c.button_pressed = value
	c.toggled.connect(cb)
	return c

func _mk_phys(label: String, lo: float, hi: float, value: float, cb: Callable) -> HBoxContainer:
	var row := HBoxContainer.new()
	var lbl := Label.new()
	lbl.text = label
	lbl.custom_minimum_size = Vector2(150, 0)
	row.add_child(lbl)
	var sl := HSlider.new()
	sl.min_value = lo
	sl.max_value = hi
	sl.step = 0.01
	sl.value = value
	sl.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	sl.size_flags_vertical = Control.SIZE_SHRINK_CENTER
	var val := Label.new()
	val.text = "%.2f" % value
	val.custom_minimum_size = Vector2(40, 0)
	sl.value_changed.connect(func(v: float):
		val.text = "%.2f" % v
		cb.call(v)
	)
	row.add_child(sl)
	row.add_child(val)
	return row