extends SceneTree
var sk: Skeleton3D
var hp: Node
func median(a: Array) -> float:
	a.sort()
	return a[a.size() / 2]
func _init():
	var scene: Node = (load("res://character_creator.tscn") as PackedScene).instantiate()
	get_root().add_child(scene)
	await process_frame
	await process_frame
	for n in scene.find_children("*", "Skeleton3D", true, false):
		if n.get_bone_count() > 200: sk = n
	hp = scene.get_node_or_null("HairPhysics")
	for i in 150: await physics_frame
	hp.wind = Vector3(15, 0, 0)
	var total := []
	var segs := [[], [], [], []]
	for i in 500:
		await physics_frame
		total.append(hp.last_solve_us / 1000.0)
		for k in 4: segs[k].append(hp.prof[k] / 1000.0)
	print("BENCH 中位耗时(ms/帧): 头骨 %.2f | 积分+约束 %.2f | 插值链+修补 %.2f | 写回 %.2f | 合计 %.2f" % [
		median(segs[0]), median(segs[1]), median(segs[2]), median(segs[3]), median(total)])
	print("BENCH_END")
	quit()
