extends SceneTree
# 提高密度后的验证: 实际落地发丝数 / 性能 / 穿模
var scene: Node

func _init() -> void:
	Engine.max_physics_steps_per_frame = 100000
	scene = (load("res://scenes/simple_hair_demo.tscn") as PackedScene).instantiate()
	get_root().add_child(scene)
	await process_frame
	scene.set_process(false)
	var hair: XPBDHairNative = scene.hair
	print("请求发丝=%d 实际落地=%d 导向束=%d" % [
		hair.get_render_requested(), hair.get_render_count(), hair.get_guide_count()])
	# 稳定
	for f in 300:
		_sim(1.0 / 60.0)
	# 计时
	var t0 := Time.get_ticks_usec()
	for f in 120:
		_sim(1.0 / 60.0)
	var ms := (Time.get_ticks_usec() - t0) / 1000.0 / 120.0
	# 穿模
	var vs: PackedVector3Array = hair.get_positions()
	var hc: Vector3 = hair.get_head_center()
	var inside := 0
	for i in vs.size():
		if vs[i].distance_to(hc) < 0.118:
			inside += 1
	print("稳态: %.2f ms/帧 | 头内粒子=%d | 头发网格顶点=%d" % [
		ms, inside, hair.get_render_count() * (hair.get_segment_count() + 1) * 2])
	# 压力: 阵风15 + 甩头
	scene.wind_level = 3
	scene.shaking = true
	var bad := 0
	for f in 240:
		_sim(1.0 / 60.0)
		if f % 2 == 0:
			var v2: PackedVector3Array = hair.get_positions()
			var hc2: Vector3 = hair.get_head_center()
			for i in v2.size():
				if v2[i].distance_to(hc2) < 0.118:
					bad += 1
	print("阵风15+甩头 240帧: 头内累计=%d" % bad)
	quit()

func _sim(dt: float) -> void:
	scene.hair.set_wind(scene.wind_levels[scene.wind_level])
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
