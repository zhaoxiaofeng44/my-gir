extends SceneTree
var sk: Skeleton3D
var hp: Node
var headb := -1
var tip := -1

func worst() -> Array:
	var w := 0.0
	var nm := ""
	for i in sk.get_bone_count():
		var n := sk.get_bone_name(i)
		if not n.contains("hair"): continue
		var gq: Quaternion = sk.get_bone_global_pose(i).basis.get_rotation_quaternion()
		var rq: Quaternion = sk.get_bone_global_rest(i).basis.get_rotation_quaternion()
		var a := rad_to_deg(gq.angle_to(rq))
		if a > w: w = a; nm = n
	return [w, nm]

func settle(n: int) -> void:
	for i in n:
		await physics_frame

func _init():
	var scene: Node = (load("res://character_creator.tscn") as PackedScene).instantiate()
	get_root().add_child(scene)
	await process_frame
	await process_frame
	for n in scene.find_children("*", "Skeleton3D", true, false):
		if n.get_bone_count() > 200: sk = n
	hp = scene.get_node_or_null("HairPhysics")
	headb = sk.find_bone("cf_J_Head_s")
	tip = sk.find_bone("cf_J_hairBL_03")
	print("参数: stiff=%.2f drag=%.2f gravity=%.2f" % [hp.stiffness, hp.drag, hp.gravity])
	await settle(180)
	var a := worst()
	print("1) 静止(默认参数): 最大偏差=%.1f° (%s)" % [a[0], a[1]])
	# 2) 转头 40° → 滞后与回稳
	var rest_dir: Vector3 = sk.get_bone_global_rest(tip).basis.y.normalized()
	var q0: Quaternion = sk.get_bone_pose_rotation(headb)
	sk.set_bone_pose_rotation(headb, q0 * Quaternion(Vector3.UP, deg_to_rad(40.0)))
	await settle(3)
	var rigid := (sk.get_bone_global_pose(headb).basis * sk.get_bone_global_rest(headb).basis.inverse() * rest_dir).normalized()
	print("2) 转头3帧 滞后=%.1f°" % rad_to_deg(sk.get_bone_global_pose(tip).basis.y.normalized().angle_to(rigid)))
	await settle(150)
	var rigid2 := (sk.get_bone_global_pose(headb).basis * sk.get_bone_global_rest(headb).basis.inverse() * rest_dir).normalized()
	print("2) 转头稳定 滞后=%.1f°" % rad_to_deg(sk.get_bone_global_pose(tip).basis.y.normalized().angle_to(rigid2)))
	# 3) 风 → 偏移 → 关风 → 回位
	var before: Vector3 = sk.get_bone_global_pose(tip).basis.y.normalized()
	hp.wind = 8.0
	await settle(90)
	print("3) 风(8) 偏移=%.1f°" % rad_to_deg(sk.get_bone_global_pose(tip).basis.y.normalized().angle_to(before)))
	hp.wind = 0.0
	await settle(300)
	print("3) 风停 300帧 残差=%.1f°" % rad_to_deg(sk.get_bone_global_pose(tip).basis.y.normalized().angle_to(before)))
	print("HP9_END")
	quit()
