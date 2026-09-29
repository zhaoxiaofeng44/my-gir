extends Node3D
## 简单人形 + C++ 头发 + C++ 裙布 演示(自包含, 秒开, 无外部资产)
##
## 鼠标左键拖动: 旋转视角(看头发/裙布物理响应)
## 空格: 甩头(头发惯性)  B: 抓着裙摆甩(布料摆动)
## R: 重置  滚轮: 缩放

var head: MeshInstance3D
var body: MeshInstance3D
var hair: XPBDHairNative
var hair_mesh: MeshInstance3D
var skirt: XPBDClothNative
var skirt_mesh: MeshInstance3D
var cam: Camera3D
var hud: Label
var cam_yaw := 0.0
var cam_pitch := -0.15
var cam_dist := 2.6
var shaking := false
var paused := false
var model_root: Node3D

func _ready() -> void:
	# 环境
	var e := Environment.new()
	e.background_mode = Environment.BG_COLOR
	e.background_color = Color(0.12, 0.13, 0.17)
	e.ambient_light_source = Environment.AMBIENT_SOURCE_COLOR
	e.ambient_light_color = Color(0.75, 0.75, 0.8)
	e.ambient_light_energy = 0.85
	var we := WorldEnvironment.new()
	we.environment = e
	add_child(we)
	var sun := DirectionalLight3D.new()
	sun.rotation_degrees = Vector3(-50, 25, 0)
	sun.light_energy = 1.25
	add_child(sun)
	# 地面
	var floorMi := MeshInstance3D.new()
	var pm := PlaneMesh.new()
	pm.size = Vector2(6, 6)
	floorMi.mesh = pm
	floorMi.position.y = 0.0
	var fm := StandardMaterial3D.new()
	fm.albedo_color = Color(0.24, 0.27, 0.32)
	floorMi.material_override = fm
	add_child(floorMi)
	# 相机
	cam = Camera3D.new()
	add_child(cam)
	_update_cam()
	cam.current = true
	hud = Label.new()
	hud.position = Vector2(16, 16)
	add_child(hud)

	# ---- 简单人形(程序化) ----
	model_root = Node3D.new()
	model_root.name = "Human"
	add_child(model_root)

	head = MeshInstance3D.new()
	var hs := SphereMesh.new()
	hs.radius = 0.115
	hs.height = 0.23
	head.mesh = hs
	head.position = Vector3(0, 1.55, 0)
	var hm := StandardMaterial3D.new()
	hm.albedo_color = Color(0.95, 0.82, 0.72)
	head.material_override = hm
	model_root.add_child(head)

	body = MeshInstance3D.new()
	var bs := CapsuleMesh.new()
	bs.radius = 0.16
	bs.height = 1.1
	body.mesh = bs
	body.position = Vector3(0, 0.95, 0)
	var bm := StandardMaterial3D.new()
	bm.albedo_color = Color(0.3, 0.5, 0.75)
	body.material_override = bm
	model_root.add_child(body)

	# ---- C++ 头发 ----
	hair = XPBDHairNative.new()
	hair.name = "HairNative"
	add_child(hair)
	hair.set_strand_count(64)
	hair.set_segment_count(10)
	hair.set_hair_length(0.30)
	hair.set_head_radius(0.118)
	hair.set_head_center(Vector3(0, 1.55, 0))
	# 身体胶囊碰撞: 头发垂到身上不穿模(肩部1.42 -> 臀部0.55, 半径0.17)
	hair.set_body_collision(true, Vector3(0, 1.42, 0), Vector3(0, 0.55, 0), 0.17)
	hair.set_render_strands(300)
	hair.set_stiffness(0.55)
	hair.set_damping(0.09)
	hair.build()
	var hm2 := StandardMaterial3D.new()
	hm2.shading_mode = BaseMaterial3D.SHADING_MODE_PER_PIXEL
	hm2.vertex_color_use_as_albedo = true
	hm2.cull_mode = BaseMaterial3D.CULL_DISABLED
	hm2.roughness = 0.45
	hair.set_material(hm2)
	hair_mesh = MeshInstance3D.new()
	add_child(hair_mesh)

	# ---- C++ 环形裙布 ----
	skirt = XPBDClothNative.new()
	skirt.name = "SkirtNative"
	add_child(skirt)
	skirt.cols = 28
	skirt.rows = 12
	skirt.ground_y = 0.0
	skirt.enable_self_collision = true
	skirt.build()
	# 环形布局: 顶行钉在腰部半径 0.19, 裙摆外扩
	var pos: PackedVector3Array = skirt.get_positions()
	var pinned: PackedByteArray = skirt.get_pinned_flags()
	var rows := 12
	var cols := 28
	var R := 0.19
	var cy := 0.78
	for r in rows:
		for c in cols:
			var i: int = r * cols + c
			var ang: float = TAU * float(c) / float(cols)
			var t: float = float(r) / float(rows - 1)
			var rr: float = R * (1.0 + 0.25 * t)
			pos.set(i, Vector3(cos(ang) * rr, cy - 0.55 * t, sin(ang) * rr))
			pinned.set(i, 1 if r == 0 else 0)
	skirt.set_positions(pos)
	skirt.set_pinned_flags(pinned)
	var sm := StandardMaterial3D.new()
	sm.cull_mode = BaseMaterial3D.CULL_DISABLED
	sm.roughness = 0.8
	sm.albedo_color = Color(0.35, 0.42, 0.65)
	skirt.set_material(sm)
	skirt_mesh = MeshInstance3D.new()
	add_child(skirt_mesh)

func _update_cam() -> void:
	var cp: Vector3 = Vector3(
		1.35 * cos(cam_yaw) * cos(cam_pitch),
		1.45 - 1.35 * sin(cam_pitch),
		1.35 * sin(cam_yaw) * cos(cam_pitch))
	cam.position = cp
	cam.look_at(Vector3(0, 1.0, 0))

func _process(delta: float) -> void:
	if paused:
		return
	var dt: float = minf(delta, 1.0 / 30.0)
	# 甩头: 头左右快速摆动
	var head_c: Vector3 = Vector3(0, 1.55, 0)
	if shaking:
		var t := Time.get_ticks_msec() / 1000.0
		head_c.x = 0.10 * sin(t * 14.0)
		head_c.z = 0.06 * cos(t * 11.0)
		head.position.x = head_c.x
		head.position.z = head_c.z
	hair.set_head_center(head_c)
	hair.set_head_moving(shaking)
	hair.step(dt, 3)
	# 裙布步进(2 子步)
	var sh := dt / 2.0
	for ss in 2:
		skirt.sub_begin()
		skirt.sub_integrate(sh, Vector3.ZERO)
		skirt.sub_solve(sh)
		skirt.begin_contacts()
		skirt.self_collision()
		skirt.final_push_out()
		skirt.sub_recover(sh)
	hair.update_mesh()
	hair_mesh.mesh = hair.get_mesh()
	skirt.update_mesh()
	skirt_mesh.mesh = skirt.get_mesh()
	if hud:
		hud.text = "FPS %d | 头发 %d束/%d丝 | 裙布 %d顶点 | 空格甩头  B裙摆  R重置" % [
			Engine.get_frames_per_second(), hair.get_guide_count(), hair.get_render_count(),
			skirt.get_positions().size()]

func _input(event: InputEvent) -> void:
	if event is InputEventMouseButton:
		var mb: InputEventMouseButton = event
		if mb.button_index == MOUSE_BUTTON_WHEEL_UP and mb.pressed:
			cam_dist = maxf(cam_dist - 0.25, 0.8)
			_update_cam()
		elif mb.button_index == MOUSE_BUTTON_WHEEL_DOWN and mb.pressed:
			cam_dist = minf(cam_dist + 0.25, 5.0)
	elif event is InputEventMouseMotion:
		var mm: InputEventMouseMotion = event
		if mm.button_mask & MOUSE_BUTTON_MASK_LEFT:
			cam_yaw += mm.relative.x * 0.008
			cam_pitch = clampf(cam_pitch + mm.relative.y * 0.006, -0.7, 0.5)
			_update_cam()
	if event is InputEventKey and event.pressed and not event.echo:
		var k: InputEventKey = event
		if k.keycode == KEY_SPACE:
			shaking = not shaking
		elif k.keycode == KEY_B:
			# 抓着裙摆甩一下(给裙布一个横向冲量)
			var pos: PackedVector3Array = skirt.get_positions()
			var rows := 12
			var cols := 28
			for r in range(rows - 4, rows):
				for c in cols:
					var i: int = r * cols + c
					var v: PackedVector3Array = skirt.get_velocities()
					v.set(i, v[i] + Vector3(1.2, 0.4, 0.5))
					skirt.set_velocities(v)
		elif k.keycode == KEY_R:
			get_tree().reload_current_scene()
