extends SceneTree
var scene: Node

func _init() -> void:
	Engine.max_physics_steps_per_frame = 100000
	scene = (load("res://scenes/simple_hair_demo.tscn") as PackedScene).instantiate()
	get_root().add_child(scene)
	await process_frame
	scene.set_process(false)
	_run(600)
	var hair: XPBDHairNative = scene.hair
	var hvs: PackedVector3Array = hair.get_positions()
	var hc: Vector3 = Vector3(0, 1.55, 0)
	var inside := 0
	for i in hvs.size():
		if hvs[i].distance_to(hc) < 0.118:
			inside += 1
	# 精确计时: 用 Time 计 120 帧的完整步进(头发3子步+裙布2子步)
	var t0 := Time.get_ticks_usec()
	for i in 120:
		hair.step(1.0 / 60.0, 3)
		var sh: float = 1.0 / 120.0
		for ss in 2:
			skirt_step(sh)
	hair.update_mesh()
	scene.skirt.update_mesh()
	var ms := (Time.get_ticks_usec() - t0) / 1000.0 / 120.0
	print("头内粒子=%d (应0)" % inside)
	print("稳态性能: 头发(3子步)+裙布(2子步) = %.2f ms/帧" % ms)
	quit()

func skirt_step(sh: float) -> void:
	var skirt: XPBDClothNative = scene.skirt
	skirt.sub_begin()
	skirt.sub_integrate(sh, Vector3.ZERO)
	skirt.sub_solve(sh)
	skirt.begin_contacts()
	skirt.self_collision()
	skirt.final_push_out()
	skirt.sub_recover(sh)

func _run(steps: int) -> void:
	for _s in steps:
		scene.hair.step(1.0 / 60.0, 3)
		var sh: float = 1.0 / 120.0
		for ss in 2:
			skirt_step(sh)
		scene.hair.update_mesh()
		scene.skirt.update_mesh()
