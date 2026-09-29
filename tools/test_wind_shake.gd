extends SceneTree
# 甩头 + 吹风 联合压力测试(逐级加压):
#   L0 静置 | L1 微风3 | L2 强风8 | L3 强风+甩头 | L4 阵风15+甩头
# 每级审计: 头内粒子 / 身体胶囊内粒子 / 裙布穿模
var scene: Node

func _init() -> void:
	Engine.max_physics_steps_per_frame = 100000
	scene = (load("res://scenes/simple_hair_demo.tscn") as PackedScene).instantiate()
	get_root().add_child(scene)
	await process_frame
	scene.set_process(false)
	_run(300)

	_run_level("L0 静置", Vector3(0, 0, 0), false)
	_run_level("L1 微风", Vector3(3, 0.5, 2), false)
	_run_level("L2 强风", Vector3(8, 1, 5), false)
	_run_level("L3 强风+甩头", Vector3(8, 1, 5), true)
	_run_level("L4 阵风15+甩头", Vector3(15, 2, 10), true)
	# 回稳
	scene.hair.set_wind(Vector3(0, 0, 0))
	scene.shaking = false
	_run(240)
	_run_level("L5 回稳(风停甩停)", Vector3(0, 0, 0), false)
	print("========== 风晃压力测试结束 ==========")
	quit()

func _run_level(tag: String, wind: Vector3, shake: bool) -> void:
	scene.hair.set_wind(wind)
	scene.shaking = shake
	var head_in := 0
	var body_in := 0
	var worst_body := 0.0
	var skirt_bad := 0
	var frames := 180   # 3秒
	for f in frames:
		scene.hair.step(1.0 / 60.0, 3)
		var sh: float = 1.0 / 120.0
		for ss in 2:
			var skirt: XPBDClothNative = scene.skirt
			skirt.sub_begin()
			skirt.sub_integrate(sh, Vector3.ZERO)
			skirt.sub_solve(sh)
			skirt.begin_contacts()
			skirt.self_collision()
			skirt.final_push_out()
			skirt.sub_recover(sh)
		# 审计(每3帧一次, 省时)
		if f % 3 == 0:
			var hvs: PackedVector3Array = scene.hair.get_positions()
			var hc: Vector3 = Vector3(0, 1.55, 0)
			var ba: Vector3 = Vector3(0, 1.42, 0)
			var bb: Vector3 = Vector3(0, 0.55, 0)
			var br: float = 0.178
			for i in hvs.size():
				var p: Vector3 = hvs[i]
				if p.distance_to(hc) < 0.118:
					head_in += 1
				var raw: float = (p - ba).dot(bb - ba) / (bb - ba).length_squared()
				var t: float = minf(maxf(raw, 0.0), 1.0)
				var near: Vector3 = ba + (bb - ba) * t
				var d: float = p.distance_to(near)
				if t > 0.001 and t < 0.999 and d < br - 0.005:
					body_in += 1
					worst_body = maxf(worst_body, br - 0.005 - d)
	print("%s: 头内=%d 身体内=%d(最深%.1fmm)" % [tag, head_in, body_in, worst_body * 1000.0])

func _run(steps: int) -> void:
	for _s in steps:
		scene.hair.step(1.0 / 60.0, 3)
		var sh: float = 1.0 / 120.0
		for ss in 2:
			var skirt: XPBDClothNative = scene.skirt
			skirt.sub_begin()
			skirt.sub_integrate(sh, Vector3.ZERO)
			skirt.sub_solve(sh)
			skirt.begin_contacts()
			skirt.self_collision()
			skirt.final_push_out()
			skirt.sub_recover(sh)
		scene.hair.update_mesh()
		scene.skirt.update_mesh()
