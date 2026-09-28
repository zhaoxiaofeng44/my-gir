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
	for i in 60: await physics_frame
	print("CD 关节是否插进头部（正=在头里，单位 mm）")
	var worst := 0.0
	var bad := []
	for r in hp.chain_depth_report():
		if r["inside"] > 0:
			bad.append(r)
		if r["guide"]:
			worst = maxf(worst, r["head"])
	print("CD 导引链头部最深穿透 = %.1f mm" % (worst * 1000.0))
	print("CD 静止时体内关节数>0 的链 = %d 条（共 %d）" % [bad.size(), hp.chain_count()])
	for r in bad.slice(0, 12):
		print("CD   %s链%d %-6s 体内最深=%+.1fmm(关节%d) 体内关节数=%d" % [
			"导引" if r["guide"] else "插值", r["ci"], r["part"],
			r["any"] * 1000.0, r["any_j"], r["inside"]])
	print("CD_END")
	quit()
