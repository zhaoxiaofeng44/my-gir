extends SceneTree
# 独立验收：加载角色场景，跑 creator 的验收指标，输出 PASS/FAIL。
# 用法: godot --headless --path . -s tools/acceptance.gd
func _init():
	var scene: Node = (load("res://character_creator.tscn") as PackedScene).instantiate()
	get_root().add_child(scene)
	await process_frame
	await process_frame
	await process_frame
	var lines: Array = scene.get("_acc_lines")
	print("========== 头身装配验收 ==========")
	for l in lines:
		print("  ", l)
	var fail := 0
	for l in lines:
		if str(l).contains("FAIL"):
			fail += 1
	print("========== 结论: %s ==========" % ("全部 PASS" if fail == 0 else "%d 项 FAIL" % fail))
	quit()
