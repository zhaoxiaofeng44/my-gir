extends SceneTree
func _init() -> void:
	var cls = ClassDB.class_exists("XPBDClothNative")
	print("XPBDClothNative registered: ", cls)
	if cls:
		var c = ClassDB.instantiate("XPBDClothNative")
		print("instantiate ok: ", c != null)
	quit()
