extends SceneTree
# 裙布(衣服)-身体 穿模审计:
#   胶囊 = 肩(0,1.42,0) - 臀(0,0.55,0), r=0.17 (与场景配置一致)
#   只统计"真正戳进身体内部"的点: 到轴距离 < r - 3mm 容差
var scene: Node
var cap_a := Vector3(0, 1.42, 0)
var cap_b := Vector3(0, 0.55, 0)
var cap_r := 0.17

func _init() -> void:
	Engine.max_physics_steps_per_frame = 100000
	scene = (load("res://scenes/simple_hair_demo.tscn") as PackedScene).instantiate()
	get_root().add_child(scene)
	await process_frame
	scene.set_process(false)
	_run(300)
	# ---- 反向对照: 关闭胶囊碰撞, 确认审计能测出穿模(避免"假通过") ----
	scene.skirt.set_body_capsule(false, Vector3(), Vector3(), 0.17)
	_level("对照-无碰撞(应测出穿模)", 3, false, false)
	scene.skirt.set_body_capsule(true, cap_a, cap_b, 0.17)
	_run(240)
	# ---- 正式审计 ----
	_level("静置", 0, false, false)
	_level("微风", 1, false, false)
	_level("强风", 2, false, false)
	_level("阵风15", 3, false, false)
	_level("阵风15+甩头", 3, true, false)
	_level("阵风15+甩头+拖拽", 3, true, true)
	# 回稳
	scene.wind_level = 0
	scene.shaking = false
	_run(240)
	_level("回稳", 0, false, false)
	print("========== 裙布穿身审计结束 ==========")
	quit()

func _level(tag: String, wind: int, shake: bool, drag: bool) -> void:
	scene.wind_level = wind
	scene.shaking = shake
	var bad := 0
	var worst := 0.0
	var frames := 180
	for f in frames:
		if drag:
			var d: Vector3 = Vector3(sin(f * 0.09) * 0.025, 0, cos(f * 0.07) * 0.02)
			scene._apply_model_delta(d)
			# 胶囊随模型移动(与 C++ 内部 translate_model 同步)
			cap_a += d
			cap_b += d
		_sim(1.0 / 60.0)
		if f % 2 == 0:
			var r: Array = _audit()
			bad += r[0]
			worst = maxf(worst, r[1])
	var vs: PackedVector3Array = scene.skirt.get_positions()
	var rad := 0.0
	var lo := 999.0
	for i in vs.size():
		rad = maxf(rad, Vector2(vs[i].x - cap_a.x, vs[i].z - cap_a.z).length())
		lo = minf(lo, vs[i].y)
	print("%s: 穿身点=%d 最深=%.2fmm | 裙布最大半径=%.3fm(应<0.35, 塌地会>1) 最低y=%.3f" % [
		tag, bad, worst * 1000.0, rad, lo])

func _audit() -> Array:
	var vs: PackedVector3Array = scene.skirt.get_positions()
	var ab: Vector3 = cap_b - cap_a
	var len2: float = ab.length_squared()
	var thr: float = cap_r - 0.003
	var cnt := 0
	var worst := 0.0
	for i in vs.size():
		var p: Vector3 = vs[i]
		var raw: float = (p - cap_a).dot(ab) / len2
		var t: float = minf(maxf(raw, 0.0), 1.0)
		var near: Vector3 = cap_a + ab * t
		var d: float = p.distance_to(near)
		if d < thr:
			cnt += 1
			worst = maxf(worst, thr - d)
	return [cnt, worst]

func _sim(dt: float) -> void:
	scene.hair.step(dt, 3)
	scene.skirt.set_wind_v(scene.wind_levels[scene.wind_level])
	var sh: float = dt / 2.0
	for ss in 2:
		var skirt: XPBDClothNative = scene.skirt
		skirt.sub_begin()
		skirt.sub_integrate(sh, Vector3.ZERO)
		skirt.sub_solve(sh)
		skirt.begin_contacts()
		skirt.self_collision()
		skirt.final_push_out()
		skirt.collide_body_capsule()
		skirt.sub_recover(sh)
	scene.hair.update_mesh()
	scene.skirt.update_mesh()

func _run(steps: int) -> void:
	for _s in steps:
		_sim(1.0 / 60.0)
