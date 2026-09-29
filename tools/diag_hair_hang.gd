extends SceneTree
var scene: Node

func _init() -> void:
	Engine.max_physics_steps_per_frame = 100000
	scene = (load("res://scenes/simple_hair_demo.tscn") as PackedScene).instantiate()
	get_root().add_child(scene)
	await process_frame
	scene.set_process(false)
	for f in 300:
		_sim(1.0 / 60.0)
	var hair: XPBDHairNative = scene.hair
	var vs: PackedVector3Array = hair.get_positions()
	var seg: int = hair.get_segment_count() + 1
	var guides: int = hair.get_guide_count()
	# 每束: 发根 y vs 发梢 y, 判断是否垂下
	var hanging := 0
	var up := 0
	for g in guides:
		var root_y: float = vs[g * seg].y
		var tip_y: float = vs[g * seg + seg - 1].y
		if tip_y < root_y - 0.02:
			hanging += 1
		elif tip_y > root_y + 0.02:
			up += 1
	print("头发: %d束 垂下=%d 上翘=%d 发根y=%.2f 发梢y范围=[%.2f,%.2f]" % [
		guides, hanging, up, vs[0].y, _min_tip(vs, seg, guides), _max_tip(vs, seg, guides)])
	quit()

func _min_tip(vs: PackedVector3Array, seg: int, guides: int) -> float:
	var m := 999.0
	for g in guides:
		m = minf(m, vs[g * seg + seg - 1].y)
	return m

func _max_tip(vs: PackedVector3Array, seg: int, guides: int) -> float:
	var m := -999.0
	for g in guides:
		m = maxf(m, vs[g * seg + seg - 1].y)
	return m

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
