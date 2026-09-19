extends SceneTree
func _init():
	var scene: Node = (load("res://character_creator.tscn") as PackedScene).instantiate()
	get_root().add_child(scene)
	await process_frame
	await process_frame
	var sk: Skeleton3D = null
	for n in scene.find_children("*", "Skeleton3D", true, false):
		if n.get_bone_count() > 200: sk = n
	var hp: Node = scene.get_node_or_null("HairPhysics")
	hp.set_enabled(false)     # 关闭并恢复静止姿态
	await process_frame
	var worst := 0.0
	var wname := ""
	for i in sk.get_bone_count():
		var nm := sk.get_bone_name(i)
		if not nm.contains("hair"): continue
		var gq: Quaternion = sk.get_bone_global_pose(i).basis.get_rotation_quaternion()
		var rq: Quaternion = sk.get_bone_global_rest(i).basis.get_rotation_quaternion()
		var a := rad_to_deg(gq.angle_to(rq))
		if a > worst: worst = a; wname = nm
	print("禁用后静止最大偏差=%.3f° (%s)" % [worst, wname])
	# 逐骨验证公式：用静止尾点应还原静止姿态
	var max_err := 0.0
	for i in sk.get_bone_count():
		var nm := sk.get_bone_name(i)
		if not nm.contains("hair"): continue
		var rest_local: Transform3D = sk.get_bone_rest(i)
		var ch: int = -1
		for k in sk.get_bone_count():
			if sk.get_bone_parent(k) == i: ch = k; break
		if ch < 0: continue
		var axis: Vector3 = sk.get_bone_rest(ch).origin.normalized()
		var grest: Transform3D = sk.get_bone_global_rest(i)
		var p: int = sk.get_bone_parent(i)
		var pgb: Basis = sk.get_bone_global_pose(p).basis if p >= 0 else Basis()
		var want_world: Vector3 = grest.basis * axis
		var want_parent: Vector3 = (pgb.inverse() * want_world).normalized()
		var rest_dir_parent: Vector3 = (rest_local.basis * axis).normalized()
		var q_delta := Quaternion(rest_dir_parent, want_parent)
		var target: Quaternion = q_delta * rest_local.basis.get_rotation_quaternion()
		var err := rad_to_deg(target.angle_to(rest_local.basis.get_rotation_quaternion()))
		max_err = max(max_err, err)
	print("公式在静止时的最大误差=%.4f°（应≈0）" % max_err)
	print("DP2_END")
	quit()
