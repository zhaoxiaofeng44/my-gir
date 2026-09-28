extends Node3D
## 布料 + 头发 统一测试套件
## 键盘: 1-7 切换测试案例 | R 重置当前案例 | Space 暂停 | 方向键 旋转相机
##
## 案例列表:
##  1 布料基础   单布落体+岩石, 基本垂坠
##  2 多层叠放   5层布叠放, 测层间防穿模
##  3 抛掷穿层   布以15m/s射向挂帘, 测高速隧穿
##  4 头发静置   发束自然垂落(导向束+插值发丝)
##  5 甩头       头部左右快速摆动, 测头发动力学响应
##  6 头发撞球   可移动障碍球穿过发束, 测碰撞
##  7 综合 Stress 5层布 + 头发 + 移动障碍 全开

@export var auto_cycle: bool = false      # 自动巡检(每8秒切一个案例)
@export var auto_cycle_seconds: float = 8.0

var current_case := 1
var case_timer := 0.0
var paused := false
var _build_done := false

# ---- 场景节点 ----
var ground: MeshInstance3D
var cam: Camera3D
var light: DirectionalLight3D
var env_node: WorldEnvironment
var rock_root: Node3D
var rock_mesh: MeshInstance3D
var rock_col: XPBDCollider
var obstacle_mesh: MeshInstance3D
var hud: Label
var case_label: Label

# ---- 布料(原生后端) ----
var cloths: Array = []
var cloth_native: XPBDClothNative = null
var use_native_backend := true
var substeps := 4

# ---- 头发 ----
var hair: XPBDHairNative = null
var hair_mesh: MeshInstance3D

# ---- 案例描述 ----
const CASES := {
	1: "布料基础: 单布落体 + 岩石垂坠",
	2: "多层叠放: 5层布, 测层间防穿模",
	3: "抛掷穿层: 布以15m/s射向挂帘",
	4: "头发静置: 导向发束自然垂落",
	5: "甩头: 头部左右摆动, 头发动力学",
	6: "头发撞球: 障碍球穿过发束",
	7: "综合: 5层布 + 头发 + 移动障碍",
}

func _ready() -> void:
	_build_env()
	_build_hair_res()
	_load_case(1)

func _build_env() -> void:
	cam = Camera3D.new()
	cam.position = Vector3(0, 1.2, 3.2)
	cam.rotation_degrees = Vector3(-12, 0, 0)
	add_child(cam)
	cam.current = true
	light = DirectionalLight3D.new()
	light.rotation_degrees = Vector3(-45, 30, 0)
	light.light_energy = 1.2
	add_child(light)
	var e := Environment.new()
	e.background_mode = Environment.BG_COLOR
	e.background_color = Color(0.13, 0.15, 0.19)
	e.ambient_light_source = Environment.AMBIENT_SOURCE_COLOR
	e.ambient_light_color = Color(0.7, 0.7, 0.75)
	e.ambient_light_energy = 0.7
	env_node = WorldEnvironment.new()
	env_node.environment = e
	add_child(env_node)
	ground = MeshInstance3D.new()
	var pm := PlaneMesh.new()
	pm.size = Vector2(6, 6)
	ground.mesh = pm
	ground.position.y = -1.6
	var gm := StandardMaterial3D.new()
	gm.albedo_color = Color(0.25, 0.28, 0.32)
	ground.material_override = gm
	add_child(ground)
	hud = Label.new()
	hud.position = Vector2(16, 16)
	add_child(hud)
	case_label = Label.new()
	case_label.position = Vector2(16, 44)
	case_label.add_theme_font_size_override("font_size", 20)
	add_child(case_label)

func _build_hair_res() -> void:
	hair = XPBDHairNative.new()
	hair.name = "HairNative"
	add_child(hair)
	var m := StandardMaterial3D.new()
	m.shading_mode = BaseMaterial3D.SHADING_MODE_PER_PIXEL
	m.vertex_color_use_as_albedo = true
	m.cull_mode = BaseMaterial3D.CULL_DISABLED
	m.roughness = 0.55
	hair.set_material(m)
	hair_mesh = MeshInstance3D.new()
	add_child(hair_mesh)

func _clear_cloth() -> void:
	for cl in cloths:
		cl.free()
	cloths.clear()
	cloth_native = null

func _new_cloth() -> Node3D:
	if use_native_backend and ClassDB.class_exists("XPBDClothNative"):
		var n: Node3D = XPBDClothNative.new()
		cloth_native = n
		return n
	var g: Node3D = Node3D.new()
	g.set_script(load("res://scripts/xpbd_cloth.gd"))
	return g

func _add_sheet(size: Vector2, res: int, origin: Vector3, col: Color, cname: String) -> Node3D:
	var cl := _new_cloth()
	cl.name = cname
	cl.cols = res
	cl.rows = res
	cl.cloth_size = size
	cl.spawn_origin = origin
	cl.albedo = col
	add_child(cl)
	cl.build()
	cloths.append(cl)
	return cl

func _add_curtain(origin: Vector3, yaw: float, col: Color) -> Node3D:
	var cl := _new_cloth()
	cl.name = "Curtain"
	cl.cols = 16
	cl.rows = 16
	cl.cloth_size = Vector2(2.0, 1.7)
	cl.spawn_origin = origin
	cl.vertical = true
	cl.yaw = yaw
	cl.albedo = col
	add_child(cl)
	cl.build()
	cloths.append(cl)
	return cl

# ---- 头发参数设置(按案例) ----
func _setup_hair(head_c: Vector3, moving: bool, obstacle: bool, ob_c: Vector3, ob_r: float) -> void:
	if hair == null:
		return
	hair.set_strand_count(48)
	hair.set_segment_count(12)
	hair.set_hair_length(0.55)
	hair.set_head_radius(0.12)
	hair.set_head_center(head_c)
	hair.set_render_strands(240)
	hair.set_stiffness(0.7)
	hair.set_obstacle(obstacle, ob_c, ob_r)
	hair.set_head_moving(moving)
	hair.build()
	var m := StandardMaterial3D.new()
	m.shading_mode = BaseMaterial3D.SHADING_MODE_PER_PIXEL
	m.vertex_color_use_as_albedo = true
	m.cull_mode = BaseMaterial3D.CULL_DISABLED
	m.roughness = 0.55
	hair.set_material(m)
	if hair_mesh:
		hair_mesh.mesh = hair.get_mesh()

func _setup_rock(pos: Vector3, scale_f: float) -> void:
	if rock_root:
		rock_root.free()
	rock_root = Node3D.new()
	add_child(rock_root)
	rock_root.position = pos
	var col: XPBDCollider = XPBDCollider.new()
	col.root = rock_root
	col.clear()
	var defs := [
		[Vector3(0, 0, 0), 0.42], [Vector3(0.26, 0.12, 0.08), 0.30],
		[Vector3(-0.24, 0.16, -0.06), 0.28], [Vector3(0.04, -0.22, 0.14), 0.30],
		[Vector3(0.08, 0.06, -0.28), 0.27], [Vector3(-0.1, -0.08, 0.26), 0.26]]
	for d in defs:
		col.add_sphere(d[0], d[1] * scale_f)
		var mi := MeshInstance3D.new()
		var sm := SphereMesh.new()
		sm.radius = d[1] * scale_f
		sm.height = d[1] * scale_f * 2.0
		mi.mesh = sm
		mi.position = d[0] * scale_f
		var mm := StandardMaterial3D.new()
		mm.albedo_color = Color(0.45, 0.42, 0.4)
		mi.material_override = mm
		rock_root.add_child(mi)
	rock_root.force_update_transform()
	# 下发给所有布料
	for cl in cloths:
		if cl.get("set_collider_shapes") != null:
			cl.set_collider_shapes(col.pt, col.poff, col.pa, col.pb, col.pr, col.phalf, rock_root.global_transform)

# ---- 案例装载 ----
func _load_case(k: int) -> void:
	current_case = k
	_clear_cloth()
	case_timer = 0.0
	paused = false
	case_label.text = "案例 %d: %s" % [k, CASES.get(k, "?")]
	match k:
		1:
			hair.set_obstacle(false, Vector3(), 0.0)
			hair.set_head_moving(false)
			hair.visible = false
			_setup_rock(Vector3(0, 0.2, 0), 1.0)
			_add_sheet(Vector2(2, 2), 24, Vector3(0, 1.8, 0), Color(0.85, 0.3, 0.35), "Sheet")
		2:
			hair.set_obstacle(false, Vector3(), 0.0)
			hair.visible = false
			_setup_rock(Vector3(0, 0.2, 0), 1.0)
			var offs: Array = [Vector3(0, 1.6, 0), Vector3(0.12, 2.2, -0.08), Vector3(-0.1, 2.8, 0.1), Vector3(0.08, 3.4, -0.05), Vector3(-0.06, 4.0, 0.08)]
			for kk in 5:
				_add_sheet(Vector2(2.0 - 0.06 * kk, 2.0 - 0.06 * kk), 20, offs[kk], Color(0.85 - 0.1 * kk, 0.3 + 0.08 * kk, 0.35), "L%d" % (kk + 1))
		3:
			hair.visible = false
			_setup_rock(Vector3(-2.5, 0.2, 0), 1.0)   # 岩石移开
			_add_curtain(Vector3(0, 2.05, -0.6), 0.0, Color(0.3, 0.5, 0.9))
			var cl := _add_sheet(Vector2(2, 2), 24, Vector3(0, 1.2, 1.4), Color(0.85, 0.3, 0.35), "Bullet")
			# 发射: 15m/s 朝挂帘
			var vs: PackedVector3Array = cl.get_positions()
			for i in vs.size():
				vs.set(i, vs[i] + Vector3(0, -0.2, -1).normalized() * 15.0)
			cl.set_velocities(vs)
		4:
			_setup_rock(Vector3(-2.5, 0.2, 0), 1.0)
			hair.visible = true
			_setup_hair(Vector3(0, 0.55, 0), false, false, Vector3(), 0.0)
		5:
			_setup_rock(Vector3(-2.5, 0.2, 0), 1.0)
			hair.visible = true
			_setup_hair(Vector3(0, 0.55, 0), true, false, Vector3(), 0.0)
		6:
			_setup_rock(Vector3(-2.5, 0.2, 0), 1.0)
			hair.visible = true
			_setup_hair(Vector3(0, 0.35, 0), false, true, Vector3(0, 0.1, 0), 0.09)
		7:
			hair.visible = true
			_setup_rock(Vector3(0, 0.2, 0), 1.0)
			_setup_hair(Vector3(0, 1.15, 0), true, true, Vector3(0, 0.35, 0), 0.09)
			var offs: Array = [Vector3(0, 2.4, 0), Vector3(0.1, 3.0, 0.06)]
			for kk in 2:
				_add_sheet(Vector2(1.6, 1.6), 18, offs[kk], Color(0.8, 0.4, 0.5), "L%d" % (kk + 1))
	_build_done = true

func _input(event: InputEvent) -> void:
	if event is InputEventKey and event.pressed and not event.echo:
		var k: InputEventKey = event
		if k.keycode >= KEY_1 and k.keycode <= KEY_7:
			_load_case(k.keycode - KEY_1 + 1)
		elif k.keycode == KEY_R:
			_load_case(current_case)
		elif k.keycode == KEY_SPACE:
			paused = not paused

func _process(delta: float) -> void:
	if paused:
		return
	var dt: float = minf(delta, 1.0 / 30.0)
	# 障碍球运动(案例6/7): 左右往返
	if (current_case == 6 or current_case == 7) and hair:
		var t: float = Time.get_ticks_msec() / 1000.0
		var c: Vector3 = Vector3(0.35 * sin(t * 1.6), 0.1 + 0.05 * sin(t * 2.3), 0.0)
		if current_case == 7:
			c.y += 0.35
		hair.set_obstacle(true, c, 0.09)
		if obstacle_mesh == null:
			obstacle_mesh = MeshInstance3D.new()
			var sm := SphereMesh.new()
			sm.radius = 0.09
			sm.height = 0.18
			obstacle_mesh.mesh = sm
			var om := StandardMaterial3D.new()
			om.albedo_color = Color(0.9, 0.75, 0.3)
			obstacle_mesh.material_override = om
			add_child(obstacle_mesh)
		obstacle_mesh.visible = true
		obstacle_mesh.position = c
	# 甩头(案例5/7): 头部左右摆动
	if (current_case == 5 or current_case == 7) and hair:
		var t2: float = Time.get_ticks_msec() / 1000.0
		var hc: Vector3 = Vector3(0.12 * sin(t2 * 3.2), 0.55 if current_case == 5 else 1.15, 0.0)
		hair.set_head_center(hc)
		hair.set_head_moving(true)
		if current_case == 7 and hair_mesh:
			pass
	# 碰撞体每帧下发给原生布料(形状数据很小, 变换可能每帧变化)
	if rock_col and rock_root:
		for cl in cloths:
			if cl.get("native") != null:
				cl.native.set_collider_shapes(rock_col.pt, rock_col.poff, rock_col.pa, rock_col.pb, rock_col.pr, rock_col.phalf, rock_root.global_transform)
	# 布料步进
	for cl in cloths:
		cl.sub_begin()
		cl.sub_integrate(dt / substeps, Vector3.ZERO)
		cl.sub_solve(dt / substeps)
		cl.begin_contacts()
		cl.self_collision()
		cl.object_collision()
		cl.final_push_out()
		cl.sub_recover(dt / substeps)
		cl.update_mesh()
		if cloth_native:
			pass
	# 布间(每帧一次)
	if cloths.size() >= 2 and cloth_native:
		cloth_native.cross_contact_all(true)
	# 头发步进
	if hair:
		hair.step(dt, 4)
		hair.update_mesh()
		if hair_mesh:
			hair_mesh.mesh = hair.get_mesh()
	case_timer += delta
	if auto_cycle and case_timer > auto_cycle_seconds:
		_load_case(current_case % 7 + 1)
	# HUD
	if hud:
		var pen := 0
		for cl in cloths:
			pen += cl.penetration_count()
		hud.text = "FPS %d | 案例%d | 布料%d 穿透%d | 布间欠量 %.2fmm" % [
			Engine.get_frames_per_second(), current_case, cloths.size(), pen,
			(cloth_native.cross_deficit_all() * 1000.0) if cloth_native else 0.0]
