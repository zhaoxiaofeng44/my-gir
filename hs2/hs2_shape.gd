# hs2_shape.gd — myHs2 web/js/hs2core/shape.js 的 GDScript 1:1 移植
# 滑杆值(0..1) → 骨骼增量 {bone: {pos:Vector3, rot:Vector3(度), scale:Vector3}}
# 公式与 Python/JS 版逐行一致：
#   keyFloat = clamp01(slider) * (KEY_COUNT-1)
#   pos = (interp - neutral) / POS_DIVISOR   rot = NormalizeAngle(diff)   scale = interp/neutral
#   多分类共享骨：pos/rot 累加、scale 累乘；mirror 时主骨累加两次（与原版一致）
class_name HS2Shape

const POS_DIVISOR := 100.0
const NEUTRAL_KEY := 12
const KEY_COUNT := 25
const EPS := 1e-4

const SUFFIX_SET := ["h", "s", "l", "r", "a", "d", "t", "sx", "sy", "sz", "rx", "ry", "rz", "tx", "ty", "tz", "ss", "ss_02", "ss_03", "ss_02sz", "ss_03sz", "ss_ty", "ty"]

var cat_map: Dictionary = {}    # catId(int) -> Array of [oldBone, mask(9 bool)]
var frames: Dictionary = {}     # oldBone -> Array of 9-float rows
var mapping: Dictionary = {}    # oldBone -> [newBone, mirrors...]

static func _strip_suffix_variants(name_in: String) -> String:
	var n := name_in
	while true:
		var last := n.rfind("_")
		if last <= 0:
			return n
		var suffix := n.substr(last + 1).to_lower()
		if suffix in SUFFIX_SET:
			n = n.substr(0, last)
		else:
			return n
	return n

# C# ConvertOldBoneNameToNewDynamic 的移植（csv 未覆盖时兜底）
static func old_to_new_dynamic(old_name: String) -> String:
	if old_name.contains("_dam"):
		return old_name
	if not old_name.begins_with("cf_s_"):
		return old_name
	var wp := old_name.substr(5)
	var lr := ""
	for suf in ["_L", "_R", "_a", "_b"]:
		if wp.ends_with(suf):
			lr = suf
			wp = wp.substr(0, wp.length() - suf.length())
			break
	var base := _strip_suffix_variants(wp)
	if base == "height":
		return "cf_N_height"
	if base.begins_with("sk_"):
		return "cf_J_" + base + lr
	if base.begins_with("hit_"):
		return "cf_" + base + lr
	return "cf_J_" + base + "_s" + lr

static func parse_keyframes(text: String) -> Dictionary:
	var out := {}
	for raw in text.split("\n"):
		var line := raw.strip_edges(false, true)
		var parts := line.split("\t")
		if parts.size() < 11:
			continue
		var bone := parts[0].strip_edges()
		if bone.is_empty():
			continue
		var n := int((parts.size() - 1) / 10.0)
		var rows: Array = []
		for k in n:
			var base := 1 + k * 10
			if base + 9 >= parts.size():
				break
			var row := []
			var ok := true
			for f in 9:
				var v_str := parts[base + 1 + f]
				if not v_str.is_valid_float():
					ok = false
					break
				row.append(v_str.to_float())
			if ok:
				rows.append(row)
		if rows.size() > 0:
			out[bone] = rows
	return out

static func parse_category_map(text: String) -> Dictionary:
	var out := {}
	for raw in text.split("\n"):
		var parts := raw.strip_edges(false, true).split("\t")
		if parts.size() < 11:
			continue
		if not parts[0].strip_edges().is_valid_int():
			continue
		var cid := int(parts[0].strip_edges())
		var bone := parts[1].strip_edges()
		if bone.is_empty():
			continue
		var mask := []
		for m in 9:
			var idx := 2 + m
			var v: String = parts[idx].strip_edges() if idx < parts.size() else ""
			mask.append(v != "" and v != "0")
		if not out.has(cid):
			out[cid] = []
		out[cid].append([bone, mask])
	return out

static func parse_mapping_csv(text: String) -> Dictionary:
	var out := {}
	var clean := text
	if clean.begins_with("\ufeff"):
		clean = clean.substr(1)
	var lines: PackedStringArray = clean.split("\n")
	for raw in lines:
		var parts := raw.split(",")
		if parts.size() < 2 or parts[0].strip_edges() == "OldBoneName":
			continue
		var arr := [parts[1].strip_edges()]
		for i in range(2, parts.size()):
			arr.append(parts[i].strip_edges())
		out[parts[0].strip_edges()] = arr
	return out

static func normalize_angle_deg(d: float) -> float:
	return fposmod(d + 180.0, 360.0) - 180.0

static func interpolate_keyframe(frames: Array, slider_t: float) -> Array:
	var key_float := clampf(slider_t, 0.0, 1.0) * (KEY_COUNT - 1)
	var lo_i: int = mini(int(floor(key_float)), frames.size() - 2)
	var lerp_t := key_float - lo_i
	var lo: Array = frames[lo_i]
	var hi: Array = frames[lo_i + 1]
	var res := [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
	for i in 9:
		if i >= 3 and i <= 5:
			var d: float = hi[i] - lo[i]
			res[i] = lo[i] + normalize_angle_deg(d) * lerp_t
		else:
			res[i] = lo[i] + (hi[i] - lo[i]) * lerp_t
	return res

func resolve(old_bone: String) -> Array:
	if mapping.has(old_bone):
		var arr: Array = mapping[old_bone]
		return [arr[0], arr.slice(1)]
	return [old_to_new_dynamic(old_bone), []]

# sliders: {catId(String或int): 0..1} → {bone: {"pos": Vector3, "rot": Vector3(度), "scale": Vector3}}
func evaluate(sliders: Dictionary) -> Dictionary:
	var pos_acc := {}
	var rot_acc := {}
	var scale_acc := {}
	for cid_str in sliders.keys():
		var val: float = sliders[cid_str]
		var cid := int(str(cid_str).to_int())
		var rows: Array = cat_map.get(cid, [])
		for row in rows:
			var old_bone: String = row[0]
			var mask: Array = row[1]
			if not frames.has(old_bone):
				continue
			var fr: Array = frames[old_bone]
			if fr.is_empty():
				continue
			var interp := interpolate_keyframe(fr, val)
			var neutral: Array = fr[mini(NEUTRAL_KEY, fr.size() - 1)]
			var res := resolve(old_bone)
			var new_bone: String = res[0]
			var mirrors: Array = res[1]
			var name_lists: Array = []
			if mirrors.size() > 0:
				name_lists.append([new_bone])
				var both := [new_bone]
				both.append_array(mirrors)
				name_lists.append(both)
			else:
				name_lists.append([new_bone])
			for name_list in name_lists:
				for tgt in name_list:
					if mask[0] or mask[1] or mask[2]:
						if not pos_acc.has(tgt):
							pos_acc[tgt] = [0.0, 0.0, 0.0]
						var d: Array = pos_acc[tgt]
						for i in 3:
							if mask[i]:
								d[i] += (interp[i] - neutral[i]) / POS_DIVISOR
					if mask[3] or mask[4] or mask[5]:
						if not rot_acc.has(tgt):
							rot_acc[tgt] = [0.0, 0.0, 0.0]
						var r: Array = rot_acc[tgt]
						for i in 3:
							if mask[3 + i]:
								r[i] += normalize_angle_deg(interp[3 + i] - neutral[3 + i])
					if mask[6] or mask[7] or mask[8]:
						if not scale_acc.has(tgt):
							scale_acc[tgt] = [1.0, 1.0, 1.0]
						var sc: Array = scale_acc[tgt]
						for i in 3:
							if mask[6 + i]:
								var n: float = neutral[6 + i]
								var ns: float = n if absf(n) > EPS else 1.0
								sc[i] *= interp[6 + i] / ns
	var out := {}
	var all_bones := {}
	for k in pos_acc.keys():
		all_bones[k] = true
	for k in rot_acc.keys():
		all_bones[k] = true
	for k in scale_acc.keys():
		all_bones[k] = true
	for b in all_bones.keys():
		var pv: Array = pos_acc.get(b, [0.0, 0.0, 0.0])
		var rv: Array = rot_acc.get(b, [0.0, 0.0, 0.0])
		var sv: Array = scale_acc.get(b, [1.0, 1.0, 1.0])
		out[b] = {
			"pos": Vector3(pv[0], pv[1], pv[2]),
			"rot": Vector3(rv[0], rv[1], rv[2]),
			"scale": Vector3(sv[0], sv[1], sv[2]),
		}
	return out

# 便捷构造：从数据目录读取四个表
static func load_from_dir(dir: String) -> Dictionary:
	var face := HS2Shape.new()
	face.cat_map = parse_category_map(FileAccess.get_file_as_string(dir + "/cf_customhead.txt"))
	face.frames = parse_keyframes(FileAccess.get_file_as_string(dir + "/cf_anmShapeHead.txt"))
	face.mapping = parse_mapping_csv(FileAccess.get_file_as_string(dir + "/face_bone_mappings.csv"))
	var body := HS2Shape.new()
	body.cat_map = parse_category_map(FileAccess.get_file_as_string(dir + "/cf_custombody.txt"))
	body.frames = parse_keyframes(FileAccess.get_file_as_string(dir + "/cf_anmShapeBody.txt"))
	body.mapping = parse_mapping_csv(FileAccess.get_file_as_string(dir + "/bone_mappings.csv"))
	return {"face": face, "body": body}
