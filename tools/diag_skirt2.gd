extends SceneTree
var scene: Node

func _init() -> void:
	Engine.max_physics_steps_per_frame = 100000
	scene = (load("res://scenes/simple_hair_demo.tscn") as PackedScene).instantiate()
	get_root().add_child(scene)
	await process_frame
	scene.set_process(false)
	var skirt: XPBDClothNative = scene.skirt
	# 构建后立即检查(未模拟)
	var p0: PackedVector3Array = skirt.get_positions()
	var pin0: PackedByteArray = skirt.get_pinned_flags()
	var ones := 0
	for i in pin0.size():
		if pin0[i] == 1:
			ones += 1
	print("模拟前: 顶点=", p0.size(), " 钉住数=", ones)
	print("模拟前 顶点0=", p0[0], " 顶点11=", p0[11], " 顶点12=", p0[12])
	# 模拟 300 帧
	for f in 300:
		_sim(1.0 / 60.0)
	var p1: PackedVector3Array = skirt.get_positions()
	var pin1: PackedByteArray = skirt.get_pinned_flags()
	var lo := 999.0
	var hi := -999.0
	var rad_max := 0.0
	for i in p1.size():
		lo = minf(lo, p1[i].y)
		hi = maxf(hi, p1[i].y)
		rad_max = maxf(rad_max, Vector2(p1[i].x, p1[i].z).length())
	print("300帧后: y范围=[", snappedf(lo, 0.01), ",", snappedf(hi, 0.01), "] 最大半径=", snappedf(rad_max, 0.01))
	print("钉住顶点0位置=", p1[0], " (应约 (0.19,0.78,0))")
	print("钉住顶点1位置=", p1[1])
	quit()

func _sim(dt: float) -> void:
	scene.hair.step(dt, 3)
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
