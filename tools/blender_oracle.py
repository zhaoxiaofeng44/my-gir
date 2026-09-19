import bpy, sys, math
from mathutils import Vector, Matrix
argv = sys.argv[sys.argv.index("--") + 1:]
fbx_dir = argv[0]
name = argv[1]          # head / body
bpy.ops.wm.read_factory_settings(use_empty=True)
bpy.ops.import_scene.fbx(filepath=fbx_dir + "/" + name + ".fbx")
# 打印所有网格对象的世界空间包围盒（Blender 世界系）
for ob in bpy.data.objects:
    if ob.type != 'MESH':
        continue
    mw = ob.matrix_world.copy()
    mn = Vector((1e9, 1e9, 1e9)); mx = Vector((-1e9, -1e9, -1e9))
    for v in ob.data.vertices:
        w = mw @ v.co
        for i in range(3):
            mn[i] = min(mn[i], w[i]); mx[i] = max(mx[i], w[i])
    print("ORACLE_%s mesh=%s world_matrix=%s" % (name, ob.name, [round(x, 6) for row in mw for x in row]))
    print("ORACLE_%s AABB_min=%s AABB_max=%s" % (name, [round(x, 6) for x in mn], [round(x, 6) for x in mx]))
print("ORACLE_DONE")
