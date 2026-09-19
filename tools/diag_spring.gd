extends SceneTree
func _init():
	var sim := SpringBoneSimulator3D.new()
	get_root().add_child(sim)
	print("初始 setting_count=", sim.setting_count)
	sim.setting_count = 7
	print("设为 7 后 setting_count=", sim.setting_count)
	sim.set_root_bone_name(0, "cf_J_hairF_top")
	sim.set_end_bone_name(0, "cf_J_hairFL_01")
	print("root0=", sim.get_root_bone_name(0), " end0=", sim.get_end_bone_name(0), " joints=", sim.get_joint_count(0))
	# 看 setting_count 属性是否可写
	for p in ClassDB.class_get_property_list("SpringBoneSimulator3D"):
		if str(p["name"]) == "setting_count":
			print("setting_count usage=", p["usage"], " (2=storage? 6=storage+editor?)")
	# 在真实骨架里试
	var scene: Node = (load("res://character_creator.tscn") as PackedScene).instantiate()
	get_root().add_child(scene)
	await process_frame
	await process_frame
	var sk: Skeleton3D = null
	for n in scene.find_children("*", "Skeleton3D", true, false):
		if n.get_bone_count() > 200: sk = n
	var sim2: Node = sk.get_node_or_null("HairSpring")
	await process_frame
	print("场景内 sim setting_count=", sim2.setting_count if sim2 else -1)
	if sim2:
		print("  root0=", sim2.get_root_bone_name(0), " joints0=", sim2.get_joint_count(0))
	print("SP2_END")
	quit()
