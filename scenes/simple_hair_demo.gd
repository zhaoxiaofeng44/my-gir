extends Node3D
## 简单人形 + C++ 头发 + C++ 裙布 (拖拽/吹风/参数调节 演示)
##
## 鼠标左键拖空白处: 旋转视角
## 鼠标左键按住人形拖动: 拖拽模型(头发/裙布物理跟随, 有惯性)
## 空格: 甩头   B: 裙摆冲量   W: 风力档位   R: 重置
## 1/2/3: 头发软硬  4/5: 裙布软硬

var head: MeshInstance3D
var body: MeshInstance3D
var hair: XPBDHairNative
var hair_mesh: MeshInstance3D
var skirt: XPBDClothNative
var skirt_mesh: MeshInstance3D
var cam: Camera3D
var hud: Label

# 相机
var cam_yaw := 0.0
var cam_pitch := -0.12
var cam_dist := 2.6

# 状态
var shaking := false
var wind_level := 0
var wind_levels: Array = [Vector3(0,0,0), Vector3(3,0.5,2), Vector3(8,1,5), Vector3(15,2,10)]
var wind_names: Array = ["无风", "微风", "强风", "阵风"]

# 模型位置(拖拽)
var model_pos := Vector3(0, 0, 0)   # 水平面上的模型位置
var model_vel := Vector3()
var dragging_model := false
var drag_plane_y := 0.9             # 拖拽平面高度(胸口)
var drag_offset := Vector3()

# 参数(1/2/3 头发, 4/5 裙布)
var hair_stiff := 0.55
var skirt_stiff := 1.0   # 占位(裙布用 stiffness 参数)

func _ready() -> void:
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
	var floorMi := MeshInstance3D.new()
	var pm := PlaneMesh.new()
	pm.size = Vector2(6, 6)
	floorMi.mesh = pm
	var fm := StandardMaterial3D.new()
	fm.albedo_color = Color(0.24, 0.27, 0.32)
	floorMi.material_override = fm
	add_child(floorMi)
	cam = Camera3D.new()
	add_child(cam)
	_update_cam()
	cam.current = true
	hud = Label.new()
	hud.position = Vector2(16, 16)
	add_child(hud)

	# ---- 人形 ----
	model_root_add()

	# ---- C++ 头发 ----
	hair = XPBDHairNative.new()
	add_child(hair)
	hair.set_strand_count(64)
	hair.set_segment_count(10)
	hair.set_hair_length(0.30)
	hair.set_head_radius(0.118)
	hair.set_head_center(Vector3(0, 1.55, 0))
	hair.set_render_strands(300)
	hair.set_stiffness(hair_stiff)
	hair.set_damping(0.09)
	hair.set_body_collision(true, Vector3(0, 1.42, 0), Vector3(0, 0.55, 0), 0.17)
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
	add_child(skirt)
	skirt.cols = 28
	skirt.rows = 12
	skirt.ground_y = 0.0
	skirt.enable_self_collision = true
	skirt.build()
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
	# 关键: 环形布局与 build() 的网格间距不同, 必须重算静止长度,
	# 否则求解器会把裙布撑开去满足旧静止长度(实测被拉长6倍摊到地上)
	skirt.refresh_rest_lengths()
	# 身体胶囊碰撞: 裙布不穿进身体(与头发同一颗胶囊, 拖拽时一同移动)
	skirt.set_body_capsule(true, Vector3(0, 1.42, 0), Vector3(0, 0.55, 0), 0.17)
	var sm := StandardMaterial3D.new()
	sm.cull_mode = BaseMaterial3D.CULL_DISABLED
	sm.roughness = 0.8
	sm.albedo_color = Color(0.35, 0.42, 0.65)
	skirt.set_material(sm)
	skirt_mesh = MeshInstance3D.new()
	add_child(skirt_mesh)

func model_root_add() -> void:
	head = MeshInstance3D.new()
	var hs := SphereMesh.new()
	hs.radius = 0.115
	hs.height = 0.23
	head.mesh = hs
	head.position = Vector3(0, 1.55, 0)
	var hm := StandardMaterial3D.new()
	hm.albedo_color = Color(0.95, 0.82, 0.72)
	head.material_override = hm
	add_child(head)
	body = MeshInstance3D.new()
	var bs := CapsuleMesh.new()
	bs.radius = 0.16
	bs.height = 1.1
	body.mesh = bs
	body.position = Vector3(0, 0.95, 0)
	var bm := StandardMaterial3D.new()
	bm.albedo_color = Color(0.3, 0.5, 0.75)
	body.material_override = bm
	add_child(body)

func _update_cam() -> void:
	cam.position = Vector3(
		cam_dist * cos(cam_yaw) * cos(cam_pitch),
		1.45 - cam_dist * sin(cam_pitch),
		cam_dist * sin(cam_yaw) * cos(cam_pitch))
	cam.look_at(Vector3(0, 1.0, 0))

func _process(delta: float) -> void:
	var dt: float = minf(delta, 1.0 / 30.0)
	# 拖拽: 模型向目标位置弹性移动(产生速度 -> 头发/裙布继承惯性)
	var target := model_pos
	var cur := Vector3(head.position.x, 0, head.position.z)
	var to_t: Vector3 = (target - cur) * 12.0 * dt
	if to_t.length() > 0.5 * dt * 60.0:
		to_t = to_t.normalized() * 0.5 * dt * 60.0
	_apply_model_delta(to_t)
	# 甩头
	var head_c: Vector3 = Vector3(head.position.x, 1.55, head.position.z)
	if shaking:
		var t := Time.get_ticks_msec() / 1000.0
		head_c.x += 0.10 * sin(t * 14.0)
		head_c.z += 0.06 * cos(t * 11.0)
	head.position = head_c
	hair.set_head_center(head_c)
	hair.set_head_moving(shaking)
	hair.set_wind(wind_levels[wind_level])
	hair.step(dt, 3)
	# 裙布: 风力 + 拖拽跟随
	skirt.set_wind_v(wind_levels[wind_level])
	var sh := dt / 2.0
	for ss in 2:
		skirt.sub_begin()
		skirt.sub_integrate(sh, Vector3.ZERO)
		skirt.sub_solve(sh)
		skirt.begin_contacts()
		skirt.self_collision()
		skirt.final_push_out()
		skirt.collide_body_capsule()
		skirt.sub_recover(sh)
	hair.update_mesh()
	hair_mesh.mesh = hair.get_mesh()
	skirt.update_mesh()
	skirt_mesh.mesh = skirt.get_mesh()
	if hud:
		hud.text = "FPS %d | 风:%s | 发刚:%.2f | 左键空白转视角 左键拖人形 W风 空格甩头 B裙 R重置" % [
			Engine.get_frames_per_second(), wind_names[wind_level], hair_stiff]

# 模型平移: 网格 + 头发 + 裙布 同步移动(头发有惯性, 裙布有摆动)
func _apply_model_delta(d: Vector3) -> void:
	if d.length_squared() < 1e-12:
		return
	head.position.x += d.x
	head.position.z += d.z
	body.position.x += d.x
	body.position.z += d.z
	hair.translate_follow(d)
	skirt.translate_model(d)

# ---- 交互 ----
func _unhandled_input(event: InputEvent) -> void:
	if event is InputEventMouseButton:
		var mb: InputEventMouseButton = event
		if mb.button_index == MOUSE_BUTTON_LEFT and mb.pressed:
			# 射线检测: 命中人形 -> 拖模型; 否则转视角
			var ray_o: Vector3 = cam.project_ray_origin(mb.position)
			var ray_d: Vector3 = cam.project_ray_normal(mb.position)
			if _hit_human(ray_o, ray_d):
				dragging_model = true
				drag_offset = Vector3(head.position.x, drag_plane_y, head.position.z) - _plane_hit(ray_o, ray_d)
			else:
				dragging_model = false
		elif mb.button_index == MOUSE_BUTTON_LEFT and not mb.pressed:
			dragging_model = false
		elif mb.button_index == MOUSE_BUTTON_WHEEL_UP and mb.pressed:
			cam_dist = maxf(cam_dist - 0.25, 0.8)
			_update_cam()
		elif mb.button_index == MOUSE_BUTTON_WHEEL_DOWN and mb.pressed:
			cam_dist = minf(cam_dist + 0.25, 5.0)
	elif event is InputEventMouseMotion:
		var mm: InputEventMouseMotion = event
		if dragging_model:
			var ray_o: Vector3 = cam.project_ray_origin(mm.position)
			var ray_d: Vector3 = cam.project_ray_normal(mm.position)
			var hit: Vector3 = _plane_hit(ray_o, ray_d) + drag_offset
			model_pos = Vector3(clampf(hit.x, -2.0, 2.0), 0, clampf(hit.z, -2.0, 2.0))
		elif mm.button_mask & MOUSE_BUTTON_MASK_LEFT:
			cam_yaw += mm.relative.x * 0.008
			cam_pitch = clampf(cam_pitch + mm.relative.y * 0.006, -0.7, 0.5)
			_update_cam()

func _hit_human(ray_o: Vector3, ray_d: Vector3) -> bool:
	# 人形包围球: 中心(模型位置, 1.0), 半径 0.85
	var c: Vector3 = Vector3(head.position.x, 1.0, head.position.z)
	var oc: Vector3 = ray_o - c
	var b: float = oc.dot(ray_d)
	var cc: float = oc.dot(oc) - 0.85 * 0.85
	var disc: float = b * b - cc
	return disc > 0.0

func _plane_hit(ray_o: Vector3, ray_d: Vector3) -> Vector3:
	# 与 y=drag_plane_y 平面求交
	var t: float = (drag_plane_y - ray_o.y) / ray_d.y
	return ray_o + ray_d * t

func _input(event: InputEvent) -> void:
	if event is InputEventKey and event.pressed and not event.echo:
		var k: InputEventKey = event
		if k.keycode == KEY_SPACE:
			shaking = not shaking
		elif k.keycode == KEY_W:
			wind_level = (wind_level + 1) % wind_levels.size()
		elif k.keycode == KEY_B:
			var vel: PackedVector3Array = skirt.get_velocities()
			for r in range(8, 12):
				for c in 28:
					var i: int = r * 28 + c
					vel.set(i, vel[i] + Vector3(1.2, 0.4, 0.5))
			skirt.set_velocities(vel)
		elif k.keycode == KEY_1:
			hair_stiff = 0.3
			hair.set_stiffness(hair_stiff)
		elif k.keycode == KEY_2:
			hair_stiff = 0.55
			hair.set_stiffness(hair_stiff)
		elif k.keycode == KEY_3:
			hair_stiff = 0.85
			hair.set_stiffness(hair_stiff)
		elif k.keycode == KEY_4:
			hair.set_damping(0.15)
		elif k.keycode == KEY_5:
			hair.set_damping(0.05)
		elif k.keycode == KEY_R:
			# 重置位置(不清场景)
			model_pos = Vector3(0, 0, 0)
