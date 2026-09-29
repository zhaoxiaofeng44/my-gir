extends SceneTree
var scene: Node

func _init() -> void:
	Engine.max_physics_steps_per_frame = 100000
	scene = (load("res://scenes/simple_hair_demo.tscn") as PackedScene).instantiate()
	get_root().add_child(scene)
	await process_frame
	scene.set_process(false)
	_run(300)
	# --- 测试1: 甩头 ---
	scene.shaking = true
	var shake_inside := 0
	for f in 120:
		scene.hair.step(1.0 / 60.0, 3)
		scene._process(1.0 / 60.0)   # 更新头位置与网格
		var vs: PackedVector3Array = scene.hair.get_positions()
		for i in vs.size():
			if vs[i].distance_to(scene.hair.get_head_center()) < 0.118:
				shake_inside += 1
	scene.shaking = false
	print("甩头120帧: 头内粒子=", shake_inside, " (应0, 甩头中头发被甩离头部属正常)")
	# --- 测试2: 裙摆冲量 ---
	var skirt: XPBDClothNative = scene.skirt
	var vel: PackedVector3Array = skirt.get_velocities()
	var rows: int = 12
	var cols: int = 28
	for r in range(rows - 5, rows):
		for c in cols:
			var i: int = r * cols + c
			vel.set(i, vel[i] + Vector3(1.5, 0.3, 0.8))
	skirt.set_velocities(vel)
	_run(120)
	# --- 审计 ---
	var hvs: PackedVector3Array = scene.hair.get_positions()
	var inside := 0
	var mx := 0.0
	for i in hvs.size():
		mx = maxf(mx, hvs[i].length())
		if hvs[i].distance_to(scene.hair.get_head_center()) < 0.118:
			inside += 1
	print("回稳后: 头内粒子=", inside, " 头发最大速度=", snappedf(mx, 0.01))
	print("物体穿透点=", scene.pen_count)
	print("裙布顶点=", skirt.get_positions().size())
	quit()

func _run(steps: int) -> void:
	for _s in steps:
		scene.hair.step(1.0 / 60.0, 3)
		var sh: float = 1.0 / 120.0
		for ss in 2:
			scene.skirt.sub_begin()
			scene.skirt.sub_integrate(sh, Vector3.ZERO)
			scene.skirt.sub_solve(sh)
			scene.skirt.begin_contacts()
			scene.skirt.self_collision()
			scene.skirt.final_push_out()
			scene.skirt.sub_recover(sh)
		scene.hair.update_mesh()
		scene.skirt.update_mesh()
