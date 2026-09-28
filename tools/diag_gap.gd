extends SceneTree
var sk: Skeleton3D
var hp: Node
func _init():
	var scene: Node = (load("res://character_creator.tscn") as PackedScene).instantiate()
	get_root().add_child(scene)
	await process_frame
	await process_frame
	for n in scene.find_children("*", "Skeleton3D", true, false):
		if n.get_bone_count() > 200: sk = n
	hp = scene.get_node_or_null("HairPhysics")
	hp.set_enabled(false)
	for i in 5: await physics_frame
	var g0: Dictionary = hp.gap_all(3)
	print("GAP 静止(无物理): 最近离体=%.1fmm 平均=%.1fmm <30mm的顶点=%d 体内=%d" % [
		g0["min_gap"] * 1000.0, g0["mean_gap"] * 1000.0, g0["near"], g0["inside"]])
	hp.set_enabled(true)
	hp.reset_physics()
	for i in 240: await physics_frame
	var g1: Dictionary = hp.gap_all(3)
	print("GAP 物理稳定后: 最近离体=%.1fmm 平均=%.1fmm <30mm的顶点=%d 体内=%d" % [
		g1["min_gap"] * 1000.0, g1["mean_gap"] * 1000.0, g1["near"], g1["inside"]])
	print("GAP 最近贴体变化 = %+.1fmm（正=被推离身体/浮空）" % ((g1["min_gap"] - g0["min_gap"]) * 1000.0))
	print("GAP_END")
	quit()
