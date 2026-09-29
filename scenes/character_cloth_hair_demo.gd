extends Node3D
## HS2 角色 + C++ 头发 + 布料衣服 可交互测试场景
##
## 鼠标左键拖动: 旋转模型(看头发/衣服的物理响应)
## 鼠标右键拖动: 平移相机
## 滚轮: 缩放
## 空格: 甩头(头部快速摆动, 看头发惯性)
## R: 重置
##
## 说明: HS2 角色自带 hair_system(GDScript 发束物理)。
## 本场景额外挂载 C++ XPBDHairNative 在头骨位置做对比演示,
## 并用 C++ XPBDClothNative 做一条"裙子"布料围在角色腰部。

var character: Node3D
var sk: Skeleton3D
var hair_native: XPBDHairNative
var hair_mesh: MeshInstance3D
var skirt: XPBDClothNative
var skirt_mesh: MeshInstance3D
var head_bone := -1
var hip_bone := -1

var cam: Camera3D
var hud: Label
var dragging := false
var panning := false
var last_mouse := Vector2.ZERO
var model_yaw := 0.0
var shake_timer := 0.0
var shaking := false
var paused := false

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
	cam = Camera3D.new()
	cam.position = Vector3(0, 1.35, 2.6)
	cam.rotation_degrees = Vector3(-8, 0, 0)
	add_child(cam)
	cam.current = true
	var floorMi := MeshInstance3D.new()
	var pm := PlaneMesh.new()
	pm.size = Vector2(8, 8)
	floorMi.mesh = pm
	floorMi.position.y = 0.0
	var fm := StandardMaterial3D.new()
	fm.albedo_color = Color(0.24, 0.27, 0.32)
	floorMi.material_override = fm
	add_child(floorMi)
	hud = Label.new()
	hud.position = Vector2(16, 16)
	add_child(hud)

	# --- HS2 角色延迟加载: 加载耗时长, 先起场景再异步构建(避免启动窗口超时) ---
	if hud:
		hud.text = "角色加载中... (首次约 10-20 秒)"
	call_deferred("_load_character_async")

func _load_character_async() -> void:
	character = Node3D.new()
	character.name = "Character"
	var script: GDScript = load("res://hs2/hs2_character.gd")
	character.set_script(script)
	add_child(character)
	# 等待骨架出现(HS2 _ready 构建合并骨架), 最多 900 帧
	var sk: Skeleton3D = null
	for i in 900:
		await get_tree().process_frame
		var stack: Array = [character]
		while stack.size() > 0:
			var n = stack.pop_back()
			for c in n.get_children():
				stack.append(c)
			if n is Skeleton3D:
				sk = n
				break
		if sk:
			break
	if sk == null:
		if hud:
			hud.text = "角色加载失败"
		return
	head_bone = sk.find_bone("cf_J_Head_s")
	if head_bone < 0:
		head_bone = sk.find_bone("cf_J_Head")
	hip_bone = sk.find_bone("cf_J_Hips")
	if hip_bone < 0:
		hip_bone = sk.find_bone("cf_J_Hips_s")
	if hud:
		hud.text = "角色就绪 (骨骼 %d)" % sk.get_bone_count()
	_setup_native_hair()
	_setup_skirt()

func _setup_native_hair() -> void:
	if not ClassDB.class_exists("XPBDHairNative"):
		return
	hair_native = XPBDHairNative.new()
	hair_native.name = "HairNative"
	add_child(hair_native)
	hair_native.set_strand_count(64)
	hair_native.set_segment_count(10)
	hair_native.set_hair_length(0.32)
	hair_native.set_head_radius(0.115)
	# 头部中心由骨骼每帧更新(见 _process)
	hair_native.set_head_center(Vector3(0, 1.55, 0))
	hair_native.set_render_strands(320)
	hair_native.set_stiffness(0.55)
	hair_native.set_damping(0.09)
	hair_native.build()
	var m := StandardMaterial3D.new()
	m.shading_mode = BaseMaterial3D.SHADING_MODE_PER_PIXEL
	m.vertex_color_use_as_albedo = true
	m.cull_mode = BaseMaterial3D.CULL_DISABLED
	m.roughness = 0.45
	hair_native.set_material(m)
	hair_mesh = MeshInstance3D.new()
	hair_mesh.name = "HairNativeMesh"
	add_child(hair_mesh)

func _setup_skirt() -> void:
	if not ClassDB.class_exists("XPBDClothNative"):
		return
	skirt = XPBDClothNative.new()
	skirt.name = "SkirtNative"
	add_child(skirt)
	# 环形裙布: cols 沿圆周, rows 垂直向下; 顶行钉在腰部
	skirt.cols = 28
	skirt.rows = 12
	skirt.cloth_size = Vector2(1.6, 0.5)
	skirt.ground_y = 0.0
	skirt.gravity = Vector3(0, -9.8, 0)
	skirt.enable_self_collision = true
	skirt.build()
	# 顶行钉住(模拟腰头)
	var pos: PackedVector3Array = skirt.get_positions()
	var pinned: PackedByteArray = skirt.get_pinned_flags()
	var rows := 12
	var cols := 28
	# build() 默认非 vertical, 顶行是 y 较小的一侧; 手动按圆环重排:
	# 用环形布局重建顶点(腰围半径 0.19)
	var R := 0.19
	var cy := 0.78
	for r in rows:
		for c in cols:
			var i: int = r * cols + c
			var ang: float = TAU * float(c) / float(cols)
			var t: float = float(r) / float(rows - 1)
			var rr: float = R * (1.0 + 0.25 * t)   # 裙摆略外扩
			var v: Vector3 = Vector3(cos(ang) * rr, cy - 0.5 * t, sin(ang) * rr)
			pos.set(i, v)
			if r == 0:
				pinned.set(i, 1)
			else:
				pinned.set(i, 0)
	skirt.set_positions(pos)
	skirt.set_pinned_flags(pinned)
	var m := StandardMaterial3D.new()
	m.shading_mode = BaseMaterial3D.SHADING_MODE_PER_PIXEL
	m.cull_mode = BaseMaterial3D.CULL_DISABLED
	m.roughness = 0.8
	m.albedo_color = Color(0.35, 0.42, 0.65)
	skirt.set_material(m)
	skirt_mesh = MeshInstance3D.new()
	skirt_mesh.name = "SkirtMesh"
	add_child(skirt_mesh)

func _process(delta: float) -> void:
	if paused:
		return
	var dt: float = minf(delta, 1.0 / 30.0)
	# 甩头: 空格按住时头部快速左右摆
	var head_c: Vector3 = Vector3(0, 1.55, 0)
	if sk != null and head_bone >= 0:
		var gp: Transform3D = sk.get_bone_global_pose(head_bone)
		# 模型根绕 X 转 180 度, 需要变换到世界
		head_c = character.global_transform * gp.origin
		# 修正: glb 的 -Y 向上约定已在 model_root 旋转里处理
		head_c.y = absf(head_c.y) + 1.45 if head_c.y < 0 else head_c.y
	if shaking:
		shake_timer += dt
		head_c.x += 0.09 * sin(shake_timer * 14.0)
		head_c.z += 0.05 * cos(shake_timer * 11.0)
	if hair_native:
		hair_native.set_head_center(head_c)
		hair_native.set_head_moving(shaking)
		hair_native.step(dt, 3)
		hair_native.update_mesh()
		hair_mesh.mesh = hair_native.get_mesh()
	# 裙布: 布料类没有 step(), 用子步序列; 环形裙布顶行已钉住, 不再跟随位移
	if skirt and skirt_mesh:
		var sh := dt / 2.0
		for ss in 2:
			skirt.sub_begin()
			skirt.sub_integrate(sh, Vector3.ZERO)
			skirt.sub_solve(sh)
			skirt.begin_contacts()
			skirt.self_collision()
			skirt.object_collision()
			skirt.final_push_out()
			skirt.sub_recover(sh)
		skirt.update_mesh()
		skirt_mesh.mesh = skirt.get_mesh()
	if hud:
		var hp := ""
		if hair_native:
			hp = " | 发束 %d" % hair_native.get_guide_count()
		hud.text = "FPS %d | 骨骼 %d%s | 左键旋转 右键平移 滚轮缩放 空格甩头 R重置" % [
			Engine.get_frames_per_second(), sk.get_bone_count() if sk else 0, hp]

# ---- 交互 ----
func _input(event: InputEvent) -> void:
	if event is InputEventMouseButton:
		var mb: InputEventMouseButton = event
		if mb.button_index == MOUSE_BUTTON_LEFT:
			dragging = mb.pressed
		elif mb.button_index == MOUSE_BUTTON_RIGHT:
			panning = mb.pressed
	elif event is InputEventMouseMotion:
		var mm: InputEventMouseMotion = event
		if dragging:
			model_yaw += mm.relative.x * 0.01
			if character:
				character.rotation.y = model_yaw
			# 相机绕模型转
			cam.rotation.y -= mm.relative.x * 0.005
			cam.rotation.x = clampf(cam.rotation.x - mm.relative.y * 0.005, -1.2, 0.6)
		elif panning:
			cam.position.y = clampf(cam.position.y - mm.relative.y * 0.003, 0.2, 2.5)
	elif event is InputEventMouseButton and event.button_index == MOUSE_BUTTON_WHEEL_UP and event.pressed:
		cam.position.z = maxf(cam.position.z - 0.25, 0.8)
	elif event is InputEventMouseButton and event.button_index == MOUSE_BUTTON_WHEEL_DOWN and event.pressed:
		cam.position.z = minf(cam.position.z + 0.25, 5.0)
	if event is InputEventKey and event.pressed and not event.echo:
		var k: InputEventKey = event
		if k.keycode == KEY_SPACE:
			shaking = not shaking
		elif k.keycode == KEY_R:
			get_tree().reload_current_scene()
