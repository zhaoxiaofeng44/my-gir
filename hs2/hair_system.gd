# hair_system.gd — HS2 自实现头发物理系统（导引发链 + 不可伸缩约束 + 身体碰撞 + 鸟群分离 + 链条插值）
#
# 为什么推倒重来：
#   旧实现直接对「头发骨骼」做尾点 verlet，但本工程的发骨 rest 位置**不在头发几何上**。
#   实测（逐骨顶点群包围盒 vs 骨 rest）：
#       cf_J_hairBL_03 骨骼 rest = (0.127, 1.755, 0.198)，而它权重覆盖的顶点渲染在 (0.14, 1.38, 0.08)
#       —— 骨链朝「头顶后上方」走，头发却朝「下前方」垂，二者相差 40cm。
#   于是绕骨头旋转顶点时，旋转中心根本不在头发上，一加风毛发就炸成碎片。
#
# 本实现的做法（每一步都有 headless 判据，见 tools/diag_hair.gd）：
#   1. 从**几何**出发建链：用「每根发骨所支配顶点的质心」作为链上关节，链序取骨骼层级序，
#      再把每条链按弧长重采样成 N 个关节 —— 于是链真正躺在头发里。
#   2. 每条链是一串**不伸缩**的质点链：Verlet 积分 + 迭代距离约束（长度恒定）。
#   3. 链条根部**钉死**（跟随头骨位姿），头发漂移/转头靠头骨增量矩阵自然带动（惯性甩动）。
#   4. 与**身体碰撞**：头（由头模 AABB 拟合的椭球）+ 颈/胸/腰（由身体顶点群拟合的椭球串），
#      全部跟随骨骼当前 pose。
#   5. 链条之间用**鸟群分离规则**（boids separation）：任意两条链的对应质点靠得太近就互相推开，
#      头发不会互相穿插、也不会黏成一坨。
#   6. **链条之间插值**：在相邻导引链之间按权重插值生成「虚拟发链」（TressFX 的 guide/interpolated 思路），
#      再把网格顶点按「到各链条折线的距离」重新分配蒙皮权重 —— 两链之间的头发由两条链混合驱动。
#
# 重绑蒙皮为什么安全：本工程头发满足 bind = rest⁻¹（见 hs2_rig_merge.gd 的 use_bind_inverse），
#   所以静止时任意归一化权重都渲染成 A·v_mesh —— 换权重/换骨头都不会改变静止外观。
class_name HS2HairSystem
extends Node

const HEAD_BONE := "cf_J_Head_s"
# 头部碰撞体段数见文件下方 HEAD_SLICES
const ROOT_F_BONE := "cf_J_hairF_top"
const ROOT_B_BONE := "cf_J_hairB_top"

# 导引发链：根骨 → 梢骨（取骨骼层级上的完整路径）
const GUIDE_DEFS := [
	{"part": "hair_f", "bones": ["cf_J_hairF_top", "cf_J_hairFL_s", "cf_J_hairFL_00", "cf_J_hairFL_01"]},
	{"part": "hair_f", "bones": ["cf_J_hairF_top", "cf_J_hairFR_s", "cf_J_hairFR_00", "cf_J_hairFR_01"]},
	{"part": "hair_f", "bones": ["cf_J_hairF_top", "cf_J_hairFSL_s", "cf_J_hairFSL_00", "cf_J_hairFSL_01", "cf_J_hairFSL_02"]},
	{"part": "hair_f", "bones": ["cf_J_hairF_top", "cf_J_hairFSR_s", "cf_J_hairFSR_00", "cf_J_hairFSR_01", "cf_J_hairFSR_02"]},
	{"part": "hair_b", "bones": ["cf_J_hairB_top", "cf_J_hairBC_s", "cf_J_hairBC_00", "cf_J_hairBC_01", "cf_J_hairBC_02", "cf_J_hairBC_03"]},
	{"part": "hair_b", "bones": ["cf_J_hairB_top", "cf_J_hairBL_s", "cf_J_hairBL_00", "cf_J_hairBL_01", "cf_J_hairBL_02", "cf_J_hairBL_03"]},
	{"part": "hair_b", "bones": ["cf_J_hairB_top", "cf_J_hairBR_s", "cf_J_hairBR_00", "cf_J_hairBR_01", "cf_J_hairBR_02", "cf_J_hairBR_03"]},
]

# 躯干/胸/肩/臂的碰撞椭球（静止姿态下的实测值；随骨骼 pose 刚体变换）
const BODY_COLLIDERS := [
	{"bone": "cf_J_Kosi02_s",  "center": Vector3(0.0, 0.999, -0.054), "half": Vector3(0.150, 0.084, 0.100)},
	{"bone": "cf_J_Kosi01_s",  "center": Vector3(0.0, 1.114, -0.024), "half": Vector3(0.134, 0.056, 0.094)},
	{"bone": "cf_J_Spine01_s", "center": Vector3(0.0, 1.205,  0.000), "half": Vector3(0.098, 0.073, 0.075)},
	{"bone": "cf_J_Spine02_s", "center": Vector3(0.0, 1.298, -0.006), "half": Vector3(0.116, 0.063, 0.083)},
	{"bone": "cf_J_Spine03_s", "center": Vector3(0.0, 1.453, -0.028), "half": Vector3(0.128, 0.091, 0.076)},
	{"bone": "cf_J_Neck_s",    "center": Vector3(0.0, 1.557, -0.032), "half": Vector3(0.046, 0.035, 0.044)},
	{"bone": "cf_J_Mune02_s_L",     "center": Vector3(0.073, 1.382, 0.086), "half": Vector3(0.052, 0.055, 0.046)},
	{"bone": "cf_J_Mune02_s_R",     "center": Vector3(-0.073, 1.382, 0.086), "half": Vector3(0.052, 0.055, 0.046)},
	{"bone": "cf_J_Shoulder02_s_L", "center": Vector3(0.108, 1.494, -0.038), "half": Vector3(0.042, 0.040, 0.062)},
	{"bone": "cf_J_Shoulder02_s_R", "center": Vector3(-0.108, 1.494, -0.038), "half": Vector3(0.042, 0.040, 0.062)},
	{"bone": "cf_J_ArmUp01_s_L",    "center": Vector3(0.190, 1.478, -0.036), "half": Vector3(0.064, 0.052, 0.056)},
	{"bone": "cf_J_ArmUp01_s_R",    "center": Vector3(-0.190, 1.478, -0.036), "half": Vector3(0.064, 0.052, 0.056)},
	{"bone": "cf_J_ArmUp02_s_L",    "center": Vector3(0.291, 1.477, -0.040), "half": Vector3(0.034, 0.039, 0.046)},
	{"bone": "cf_J_ArmUp02_s_R",    "center": Vector3(-0.291, 1.477, -0.040), "half": Vector3(0.034, 0.039, 0.046)},
	{"bone": "cf_J_ArmUp03_s_L",    "center": Vector3(0.362, 1.478, -0.048), "half": Vector3(0.028, 0.034, 0.038)},
	{"bone": "cf_J_ArmUp03_s_R",    "center": Vector3(-0.362, 1.478, -0.048), "half": Vector3(0.028, 0.034, 0.038)},
	{"bone": "cf_J_ArmLow01_s_L",   "center": Vector3(0.435, 1.477, -0.043), "half": Vector3(0.047, 0.038, 0.040)},
	{"bone": "cf_J_ArmLow01_s_R",   "center": Vector3(-0.435, 1.477, -0.043), "half": Vector3(0.047, 0.038, 0.040)},
]
# 头部碰撞体按 pose 空间网格拟合（见 _build_colliders），不用常量表
const HEAD_SLICES := 6                             # 头部沿竖直方向切成几段椭球

# ---------------- 可调参数 ----------------
@export var enabled := true
@export_range(0.0, 1.0) var stiffness := 0.0       # 离位弹簧：关。它抵抗平移=给头发装钢筋，
                                                   # 形状记忆交给弯曲约束。头发几乎没有这个力。
                                                  # 头发几乎没有这个力，主要靠下面的弯曲约束。
                                                  # 调小它 + 开弯曲约束，头发才会柔顺下垂。
                                                  # 实测调小(0.22)会让头发炸开 —— 链从头顶是**径向外指**的，
                                                  # 失去形状记忆后它们不往下垂，而是沿径向炸开。
                                                  # 要"柔软下垂"不能靠削弱形状记忆，得靠让链形本身朝下。
@export_range(0.0, 1.0) var bend_stiffness := 0.12  # 弯曲约束刚度：形状记忆的正确形式 ——
                                                   # 实测单独开它 + 调弱离位弹簧 ⇒ 头发炸开，
                                                   # 因为碰撞/鸟群的推力失去了制衡。需要与碰撞一起重整。
@export_range(0.0, 1.0) var drag := 0.70           # 阻尼（头发在空气里是重阻尼系统）
@export var gravity := 9.8                         # m/s²（世界 -Y，真实重力）
@export var wind := Vector3.ZERO                   # m/s²
@export var linear_damping := 1.0                  # 速度额外衰减（每秒，指数）
@export var substeps := 2                         # 子步进：快速转头时单步位移减半。
                                                   # 注意不能加太多 —— FTL/弯曲投影是绝对位置修正，
                                                   # 子步越多泵入能量越多（实测 3 子步比 2 子步更抖）
@export var max_joint_speed := 0.05                # 关节速度上限（米/帧）：硬截断爆炸性运动
@export var iterations := 8                         # 约束迭代次数。只有 7 条链时迭代很便宜，
                                                   # 提高它 = 碰撞有更多机会把关节推出去
@export var joints_per_chain := 16
                                                   # 但骨数 = 链数 × 关节数，性能按比例增长
@export var interpolated_per_gap := 3              # 相邻导引链之间的虚拟链条数
@export var boid_separation := 0.0                 # 鸟群分离：关。它是"炸毛"的元凶之一 ——
                                                   # 链条贴身堆叠时被持续推开，与重力/碰撞形成极限环
@export var boid_strength := 1.0
@export var collision_enabled := true
@export var collision_radius_scale := 1.0
@export var head_extra := 0.0                      # 头部碰撞体额外外扩（米）
@export var collision_margin := 0.003              # 碰撞体基础外扩（米）
@export var max_rest_push := 0.0                   # 静止预推上限（0=不做位移，只重建段长）
@export var scalp_attach_margin := 0.008           # 判定「还算贴着头皮」的容差（米）
@export var scalp_min_attach := 3
# ---- 模型顶点邻近查询（替代椭球做碰撞）----
@export var vtx_cell := 0.03                       # 顶点哈希格子边长（米）
@export var vtx_query_k := 16                      # 每个关节缓存多少个最近模型顶点
@export var vtx_margin := 0.002                    # 链条节点离模型表面的最小间距（米）
@export var vert_contact := false                  # 逐帧用 mesh 顶点做接触（实时距离）。
                                                   # 算法完整可用（见 _nearest_vertex / _vert_contact），
                                                   # 实测贴合度 11.3→6.8mm，但 GDScript 下 ~1000 个关节
                                                   # 每帧做哈希查询要 60~135ms/帧，跑不动，默认关。
@export var vert_contact_relax := 0.55             # 每次推挤的推进比例（<1 防止与链长约束打架产生抖动）
@export var contact_margin := 0.008                # 接触间隙：需略大于顶点间距的一半（~5mm），
                                                   # 实测 3/8/16mm 对穿透数几乎无影响（228~233），
                                                   # 说明瓶颈不在间隙，而在只查 16 个缓存顶点的精度。
                                                   # 原来用 rad[j]（发片厚度，最大 25mm）当推挤半径，
                                                   # 于是链心被顶离身体 25mm —— 头发看起来是浮着的。
                                                   # 物理上链心就该贴着体表，发片厚度是头发自己的体积。
                                                   # 只挡住"真的进到表面里"的节点，不硬推贴着的头发
@export var vtx_query_radius := 0.05               # 邻近顶点查询半径（米）
@export var scalp_grid_roots := true               # 发根用"头皮网格排布"生成（网状），取代骨骼链质心
@export var grid_rows := 5                         # 网格行数：头顶 → 颈后
@export var grid_cols := 5
@export var scalp_len_top := 0.18                  # 头顶行发链长度（米）
@export var scalp_len_bottom := 0.55               # 颈后行发链长度（米）
@export var chain_target := 7                      # 先只跑 7 条导引链（按"先研究链条物理，
                                                   # 最后再考虑插值"的顺序）；调稳后再加回插值链
@export var vtx_refresh_interval := 3              # 每 N 帧刷新一次顶点缓存
@export var interpolated_collision := false        # 插值(虚拟)链单独做碰撞。默认关：
                                                   # 它们由两条导引链的当前位置 lerp 而来，
                                                   # 当两链分居身体两侧时连线必然穿过身体
                                                   # （实测插值链最深陷入 93.6mm，而导引链没问题）
@export var tri_cell := 0.05                      # 三角形哈希格子边长（米）
@export var tri_query_k := 12                      # 每关节缓存多少个候选三角形
@export var tri_query_radius := 0.045              # 候选三角形的查询半径（米）
@export var tri_refresh_dist := 0.030              # 关节移动超过此距离才重查候选（米）
@export var tri_collision := false                 # 用真实网格（最近三角形 SDF）做碰撞。
                                                   # 算法已完整实现（三角形哈希 + 最短点 + 顶点法线），
                                                   # 但实测开启后穿模从 28 个恶化到 266 个非发根关节，
                                                   # 原因见 README：推挤目标距离与"不伸缩"约束互相打架，
                                                   # 且需要全量精确查询（当前 GDScript 下 >40ms/帧）。
                                                   # 算法已实现且验证有效（见 _push_out_tris_cached），
                                                   # 但 GDScript 下开销 >30ms/帧（需同时推 35 条链），
                                                   # 暂默认关，留作性能优化后启用
@export var vtx_collision := true                  # 用**缓存的模型顶点**做接触：
                                                   # 每帧只算 K 个点积（不做哈希查询），
                                                   # 既贴头皮又不穿模 —— 椭球做不到这件事。
                                                   # "邻近顶点 + 顶点法线取最大值"不是真正的 SDF，
                                                   # 会把链条逐渐推歪（静止漂移 100~160mm）
@export var head_inset := 0.90                    # 头部椭球**内切**系数：让它只当粗筛，
                                                   # 不用它把头发顶离头皮（那会导致"竖立不贴"）。
                                                   # 真正的头部接触交给缓存的 mesh 顶点。
                                                   # 外扩会顶飞贴头皮的头发，内切只挡"钻进头里"的链。
@export var head_mesh_collision := false           # 头部按真实头模（射线）推挤（已被三角形 SDF 取代）
                                                   # 实测会把"资产本来就长在头里"的内层发片推出去，
                                                   # 静止漂移 39~68mm（发型被扯开），故默认关。
# ---- 严格网格碰撞（体素占据场）：判据与渲染用同一批三角形 ----
@export var sdf_enabled := false                   # 实验性（对这套网格的实体化仍有漏洞，默认关）
@export var sdf_step := 0.008                      # 体素步长（米），越小越精确越占内存
@export var sdf_min := Vector3(-0.40, 0.95, -0.34)
@export var sdf_max := Vector3(0.40, 1.90, 0.28)
@export var mesh_repair := false                   # 网格级修补（旧椭球系统）。它跑在 _solve() **之后**，
                                                   # 用椭球把关节再推一次 ⇒ 会推翻刚做好的精确碰撞，
                                                   # 既是残余穿透也是抖动的来源。新 XPBD 求解已不需要它。
@export var mesh_repair_iterations := 1
@export var mesh_repair_relaxation := 0.7          # 每个顶点只推送穿透量的一部分，避免过冲/抖
@export var candidate_range := 0.022               # 只修补「离身体这么近」的顶点（米）
                                                   # 太小会漏：静止时离身体远的顶点，转头/甩动后可能荡进身体
@export var mesh_repair_interval := 2              # 每 N 个物理帧做一次网格修补（1 帧延迟，肉眼不可见）
@export var candidate_refresh_interval := 15       # 每 N 次修补刷新一次「最近椭球」粗排

# ---- 显示模式：0=只渲染头发网格 1=只渲染物理发链 2=两者都渲染 ----
@export var display_mode := 0
@export var chain_width := 0.0035                  # 发链根部的绘制宽度（米），向梢部渐细。
                                                   # 调细才像"发丝"，原来的 8mm 看着像"铁皮"
@export var chain_show_joints := false             # 是否画关节标记点
@export var max_hair_radius := 0.025               # 单关节「发片向内厚度」上限（米）
@export var radius_percentile := 0.9               # 取「更深入身体多少」的分位数

# ---------------- 运行态 ----------------
var _sk: Skeleton3D
var _A := Transform3D()                            # 头发装配矩阵（mesh → 骨架局部）
var _head := -1
var _head_rest_inv := Transform3D()
var _head_pose := Transform3D()                    # 本帧头骨位姿增量 M_head
var _head_pose_inv_prev := Transform3D()
var _S_prev := Transform3D()                       # 上一帧骨架全局变换（用于整体移动/旋转的惯性）
var _first_frame := true
var _g_local := Vector3(0.0, -9.8, 0.0)
var _ok := false

var _chains: Array = []        # 每条链一个 Dictionary
var _guides: Array = []        # 导引链在 _chains 中的下标
var _bones: Array = []         # 新建的骨头下标（用于清 override）
var _colliders: Array = []     # {bone, center, basis_rest, half}
var _sh_c := PackedVector3Array()  # 扁平化碰撞椭球：中心
var _sh_bi: Array = []             # 逆基
var _sh_b: Array = []              # 基
var _sh_h := PackedVector3Array()  # 半轴
var _sh_r := PackedFloat32Array()  # 半轴长度（粗筛）
var _sdf := PackedByteArray()      # 严格碰撞场：1 = 在 body/head 网格内部
var _sdf_dim := Vector3i.ZERO
var _sdf_min := Vector3.ZERO
var _sdf_ok := false
# 每条链的前 N 个关节（含根）判定为「长在头皮里」：刚性跟随头骨、不参与物理；
# 超出头皮的其余关节才做 verlet / 碰撞 / 鸟群。这就是"发根固定在头部、其余参与物理"。
var _attach := PackedInt32Array()
# 头模三角形 + 空间哈希（rest 空间）：头部碰撞只对"真的在头模内部"的点生效。
# 头部椭球比头模胖，直接拿椭球推会把"贴着头皮"的头发顶飞（实测静止漂移 88mm）。
var _head_tris: Array = []
var _head_grid: Dictionary = {}
var _head_cell := 0.04
var _head_ids := PackedInt32Array()
# 模型顶点哈希（rest 空间）：碰撞/定根都基于它，不再依赖椭球近似
var _vtx_pos := PackedVector3Array()
var _vtx_nrm := PackedVector3Array()
var _vtx_grid := {}
var _vtx_ok := false
var _joint_vtx: Array = []          # 每条链：每个关节缓存的最近顶点下标（K 个连续存放）
var _vtx_coarse := {}               # 6cm 粗占据格：快速剔除"附近根本没有 mesh 顶点"的关节
var _head_split := -1               # 顶点数组里 head 部分的起始下标
# 惰性 SDF 网格（rest 空间）：按格子缓存"最近模型顶点的有符号距离 + 外法线"。
# 身体在链条空间里静止 ⇒ 缓存永不失效；很多关节共享同一格 ⇒ 命中率高。
var _sdf_cv := {}                   # cell key -> 该格的最近顶点下标（-1 = 附近无网格）
var _body_center := Vector3.ZERO    # 身体顶点质心（深陷时的逃逸参照）
var _head_center := Vector3.ZERO    # 头部顶点质心（同上）
var _vtx_bounds := AABB()           # 模型顶点包围盒（含少量余量）：盒外必在体外
var _parity_cell := {}              # cell key -> 射线奇偶结果（身体静止 ⇒ 永久有效）
var _lam_l: Array = []              # 每链：距离约束 λ（段数）
var _lam_b: Array = []              # 每链：弯曲约束 λ（关节数）
var _lam_c: Array = []              # 每链：碰撞约束 λ（关节数）
const _sdf_cell := 0.010            # SDF 缓存格边长（米）
const _max_push := 0.040
@export var attach_compliance := 0.002             # 头皮附着柔顺度：发根柔性绑在头皮（米）
@export_range(0.0, 0.5) var neighbour_share := 0.35  # 推挤向邻居分摊的比例：越大越不拉伸、
                                                   # 但单点逃逸能力越弱。0.5 = 完全平移不改变段长
const _max_push_final := 0.004      # **收尾**那一趟推挤的上限（米）：小步长 ⇒ 不注入拉伸/不跳动。
                                   # 之前最后一步用 40mm、后面又没有长度恢复，
                                   # 段长会被拉成 250%（用户看到的"链条有弹性"）。
const _coarse_cell := 0.06
var _tmp_vtx := PackedInt32Array()
var _head_center_y := 1.66
var _head_verts := PackedVector3Array()   # 头模顶点（rest 空间），定发根用
var _crown := Vector3.ZERO                # 头顶中心（头模顶部 25% 的质心）
var _radius: Array = []        # 与 _chains 平行：每个关节的「发片到链心」厚度（米）
var _bone_rc := {}             # 骨下标 -> Vector2i(链下标, 关节下标)
var _hair_meshes: Array = []
var _parts: Dictionary = {}                        # 建/重建碰撞体时要用到的网格
var _chain_mi: MeshInstance3D                      # 发链可视化节点（挂在骨架下，与其他网格同一空间）
var _chain_im: ImmediateMesh
# 网格级修补的候选顶点（扁平数组，热路径零 Dictionary）
var _cand_m := PackedInt32Array()
var _cand_i := PackedInt32Array()
var _cand_b := PackedInt32Array()
var _cand_w := PackedFloat32Array()
var _cand_v := PackedVector3Array()
const _joint_neighbours := 8    # 碰撞体共 24 颗，按"最近"排序取前 8 颗。
                                # 取 10 时漏测会穿模，取 12 时 35 条链下太贵；
                                # 取 8 + 三角形 SDF 兜底是当前的速度/正确性折中
var _joint_near: Array = []         # 每条链：每个关节最近邻的 _joint_neighbours 颗椭球
const _cand_neighbours := 6         # 每个修补候选顶点测最近邻的几颗椭球
var _cand_n := PackedInt32Array()
var _cand_refresh := 0
var steps := 0

# 调试/验收指标
var metric_max_stretch := 0.0      # 最大相对伸长
var metric_min_sep := 999.0        # 链条间最小间距
var metric_max_pen := 0.0          # 最大身体穿透（米，蒙皮网格顶点）
var metric_pen_verts := 0          # 「静止时在体外、现在穿进身体」的顶点数（真正的穿模）
var metric_pen_at := Vector3.ZERO  # 最深穿透点
var metric_pen_bone := ""          # 最深穿透顶点的主控骨（哪条链的哪个关节）
var _metric_stride := 1
var last_solve_us := 0             # 上一物理帧求解耗时（微秒），性能自检用
var prof := [0, 0, 0, 0]           # [头骨更新, 积分+约束, 插值链, 写回] 各段微秒
var _rest_outside: Array = []      # 每个头发网格：PackedByteArray，1 = 静止时该顶点在身体外

# =====================================================================
# 构建
# =====================================================================
func build(skeleton: Skeleton3D, parts: Dictionary) -> void:
	_sk = skeleton
	_chains.clear(); _guides.clear(); _bones.clear(); _colliders.clear(); _hair_meshes.clear()
	_ok = false
	if _sk == null:
		return
	var hair_f: MeshInstance3D = parts.get("hair_f")
	var hair_b: MeshInstance3D = parts.get("hair_b")
	if hair_f == null and hair_b == null:
		push_warning("[HairSystem] 缺少头发网格")
		return
	_hair_meshes = [hair_f, hair_b].filter(func(m): return m != null)

	_parts = parts
	_head = _sk.find_bone(HEAD_BONE)
	if _head < 0:
		push_warning("[HairSystem] 找不到 " + HEAD_BONE)
		return
	_head_rest_inv = _sk.get_bone_global_rest(_head).affine_inverse()

	# A = rest(hairF_top) · bind(hairF_top) —— 头发 mesh 空间 → 骨架局部空间的装配矩阵
	var skin0: Skin = hair_f.skin if hair_f != null else hair_b.skin
	var anchor := _sk.find_bone(ROOT_F_BONE)
	if anchor < 0:
		anchor = _sk.find_bone(ROOT_B_BONE)
	_A = _sk.get_bone_global_rest(anchor) * skin0.get_bind_pose(anchor)

	# 顶点哈希必须在建链之前：定发根（头皮附着点）要用它
	_head_center_y = _sk.get_bone_global_rest(_head).origin.y
	_build_vertex_hash(parts)

	_build_tri_hash(parts)
	# 1) 采集「每根发骨支配的顶点」在骨架局部空间的质心
	var groups := _collect_hair_groups()

	# 2) 由几何生成导引链折线
	var raw: Array = []
	if scalp_grid_roots:
		# **网状发根排布**：头皮 (行,列) 网格 → 每格一条物理链。
		# 发根天然网状覆盖整个头皮，35 条链全是真实导引链（无插值链）。
		raw = _build_scalp_grid_chains()
	else:
		for d in GUIDE_DEFS:
			var poly := _chain_polyline(d["bones"], groups)
			if poly.size() >= 2:
				poly[0] = _scalp_root(poly[1])
				raw.append({"part": d["part"], "poly": poly})
	if raw.is_empty():
		push_warning("[HairSystem] 未能生成发链")
		return

	# 3) 弧长重采样 + 建立导引链
	var n: int = maxi(joints_per_chain, 4)
	for r in raw:
		var q := _resample(r["poly"], n)
		_chains.append(_make_chain(r["part"], q, true, -1, -1, 0.0))
		_guides.append(_chains.size() - 1)

	# 4) 相邻导引链之间插值出虚拟发链
	#    网状排布模式下发根本身已经网状铺开，不再需要插值链
	if not scalp_grid_roots:
		_build_interpolated(n)

	# 5) 碰撞体（要在蒙皮之前：重分配权重时会用头部碰撞体中心算"朝内发片"的假 AO）
	_build_colliders(parts)

	# 5b) 严格碰撞场（体素占据，直接从 mesh 三角形实体化）
	if sdf_enabled:
		_build_sdf(parts)

	# 5c) 建头模探针（后面推挤避让 + 固定段判定都要用）
	_build_head_probe()

	# 5d) 判定"头皮固定段"：**按资产原始链形分类**。
	#     发片内层本来就"长在头里"，这些关节必须固定住不参与物理，
	#     否则任何严格推挤都会把发型扯开（实测漂移 50~90mm）。
	_compute_attachment()

	# 6) 把静止发链推出碰撞体（只推自由段，且**不允许被推进头里**）。
	#    必须在建骨/算权重**之前**：骨 rest / 蒙皮权重都是按最终链形算的。
	_relax_rest_chains()



	# 6c) 刷新每个关节的邻近模型顶点缓存（碰撞热路径用；不用顶点碰撞就跳过）
	if vtx_collision:
		_refresh_joint_vtx()

	# 7) 建骨 + 建蒙皮 + 重分配权重 + 烘焙发丝切线/根梢参数
	_create_bones_and_skin(parts)

	# 8) 每个关节的发片厚度 + 每个关节的邻近碰撞体（粗排）
	_compute_radii()
	_compute_joint_neighbours()

	# 9) 发链可视化（可切换：只渲染物理链条）
	_build_chain_view()

	# 重力换算到骨架局部空间（世界重力为 -Y）
	var S: Transform3D = _sk.global_transform
	_g_local = (S.basis.inverse() * Vector3(0.0, -1.0, 0.0)).normalized()

	_reset()
	_compute_rest_outside()
	_collect_candidates()
	_ok = true
	print("[HairSystem] 发链 %d 条（导引 %d + 插值 %d），关节 %d，新建骨 %d，碰撞体 %d，修补候选顶点 %d" % [
		_chains.size(), _guides.size(), _chains.size() - _guides.size(),
		_chains.size() * n, _bones.size(), _colliders.size(), _cand_m.size()])

func _make_chain(part: String, q: PackedVector3Array, guide: bool, ga: int, gb: int, blend: float) -> Dictionary:
	var lens := PackedFloat32Array()
	var total := 0.0
	for i in q.size() - 1:
		var l := q[i].distance_to(q[i + 1])
		lens.append(l); total += l
	# 弯曲约束的静止量：|q[j-1] - q[j+1]|。它不是"形状记忆弹簧"，
	# 而是 PBD 里的**距离约束**（约束两端点），只抵抗弯曲、不抵抗平移 ——
	# 这才是头发"柔顺但有连续性"的物理来源。原来的位置弹簧是抵抗平移的，
	# 等于给每根头发装了钢筋，所以它们会"立起来"。
	var bend := PackedFloat32Array()
	bend.resize(q.size())
	for j in q.size():
		if j >= 1 and j + 1 < q.size():
			bend[j] = q[j - 1].distance_to(q[j + 1])
		else:
			bend[j] = 0.0
	var p := q.duplicate()
	return {
		"part": part, "q": q, "p": p, "prev": p.duplicate(), "target": q.duplicate(),
		"len": lens, "total": total, "guide": guide, "bend": bend,
		"ga": ga, "gb": gb, "blend": blend, "bones": PackedInt32Array(),
	}

# 相邻导引链之间插值出虚拟发链

# =====================================================================
# 模型顶点空间哈希 —— 本系统所有"贴着身体"判定的基础
# ---------------------------------------------------------------------
# 椭球碰撞的根本缺陷：它只是近似，肩/头顶这些形状它盖不准（实测链条照样穿肩、穿头顶）。
# 直接逐三角形判定又太贵（每点要测上百个三角形）。
#
# 这里把 body+head 的**蒙皮顶点**装进 3cm 均匀网格：
#   · 「链条节点 ↔ 模型顶点」最近邻查询 = 只查邻近 27 个格子 → O(1)；
#   · 每个关节缓存最近 K 个顶点（隔若干帧刷新一次），逐帧碰撞只遍历这 K 个；
#   · 用顶点法线做有符号距离：sd = dot(p - v, n_v)，sd < margin 就沿 n_v 推出去。
# 这样既精确到网格（不再是椭球近似），又足够快（每帧只做几千次点积）。
# =====================================================================
func _build_vertex_hash(parts: Dictionary) -> void:
	_vtx_pos.clear()
	_vtx_nrm.clear()
	_vtx_grid.clear()
	_vtx_ok = false
	var cell: float = vtx_cell
	for key2 in ["body", "head"]:
		var mi: MeshInstance3D = parts.get(key2)
		if mi == null or mi.skin == null:
			continue
		var skin: Skin = mi.skin
		var nb: int = mini(_sk.get_bone_count(), skin.get_bind_count())
		var mats := []
		mats.resize(nb)
		for b in nb:
			mats[b] = _sk.get_bone_global_rest(b) * skin.get_bind_pose(b)
		for surf in mi.mesh.get_surface_count():
			var arr: Array = mi.mesh.surface_get_arrays(surf)
			var v: PackedVector3Array = arr[Mesh.ARRAY_VERTEX]
			var nrm: PackedVector3Array = arr[Mesh.ARRAY_NORMAL]
			var bo: PackedInt32Array = arr[Mesh.ARRAY_BONES]
			var we: PackedFloat32Array = arr[Mesh.ARRAY_WEIGHTS]
			for i in v.size():
				var acc := Vector3.ZERO
				var ws := 0.0
				for k in 4:
					var w: float = we[i * 4 + k]
					if w <= 0.0:
						continue
					var bi: int = bo[i * 4 + k]
					if bi >= nb:
						continue
					acc += (mats[bi] as Transform3D) * v[i] * w
					ws += w
				if ws <= 0.0:
					continue
				var p := acc / ws
				# 只保留头发可能触及的区域
				if p.y < 0.95 or absf(p.x) > 0.45 or absf(p.z) > 0.36:
					continue
				var n: Vector3 = nrm[i] if nrm.size() > i else Vector3.UP
				if n.length_squared() < 1e-8:
					n = Vector3.UP
				var idx := _vtx_pos.size()
				_vtx_pos.append(p)
				_vtx_nrm.append(n.normalized())
				if key2 == "head":
					_head_verts.append(p)
					_head_split = idx
				var gx := int(floor(p.x / cell))
				var gy := int(floor(p.y / cell))
				var gz := int(floor(p.z / cell))
				var key := gx * 1000003 + gy * 10007 + gz
				if not _vtx_grid.has(key):
					_vtx_grid[key] = PackedInt32Array()
				var bkt: PackedInt32Array = _vtx_grid[key]
				bkt.append(idx)
				_vtx_grid[key] = bkt
	# 统一法线朝向为"向外"。
	# 这套资产的三角形绕向不一致，ARRAY_NORMAL 也因此不可信；
	# 用"各部位质心"做参照：对近似凸的躯干/头，dot(n, v-质心)>0 即外向。
	# 有符号距离的符号全靠它 —— 法线反了碰撞就会把点往身体里推。
	if _vtx_pos.size() > 0:
		var nb_i: int = _head_split if _head_split >= 0 else _vtx_pos.size()
		var cbody := Vector3.ZERO
		var chead := Vector3.ZERO
		for i in nb_i:
			cbody += _vtx_pos[i]
		for i2 in range(nb_i, _vtx_pos.size()):
			chead += _vtx_pos[i2]
		cbody = cbody / float(maxi(nb_i, 1))
		chead = chead / float(maxi(_vtx_pos.size() - nb_i, 1))
		_body_center = cbody
		_head_center = chead
		var flipped := 0
		for i3 in _vtx_pos.size():
			var ref: Vector3 = chead if i3 >= nb_i else cbody
			if _vtx_nrm[i3].dot(_vtx_pos[i3] - ref) < 0.0:
				_vtx_nrm[i3] = -_vtx_nrm[i3]
				flipped += 1
		print("[HairSystem] 法线统一外向：翻转 %d / %d" % [flipped, _vtx_pos.size()])
	# 包围盒（外扩 12mm）：盒外的关节必在体外，连奇偶判定都不用做
	if _vtx_pos.size() > 0:
		var bmin := _vtx_pos[0]
		var bmax := _vtx_pos[0]
		for vv in _vtx_pos:
			bmin = bmin.min(vv)
			bmax = bmax.max(vv)
		_vtx_bounds = AABB(bmin - Vector3.ONE * 0.012, (bmax - bmin) + Vector3.ONE * 0.024)
	_vtx_coarse.clear()
	for p2 in _vtx_pos:
		var kx := int(floor(p2.x / _coarse_cell))
		var ky := int(floor(p2.y / _coarse_cell))
		var kz := int(floor(p2.z / _coarse_cell))
		_vtx_coarse[kx * 1000003 + ky * 10007 + kz] = true
	_vtx_ok = _vtx_pos.size() > 0
	# 头顶中心：头模顶部 25% 高度的质心 —— 定发根时的"原点"
	_crown = Vector3.ZERO
	if _head_verts.size() > 0:
		var ymin := INF
		var ymax := -INF
		for v in _head_verts:
			ymin = minf(ymin, v.y)
			ymax = maxf(ymax, v.y)
		var ycut: float = ymin + (ymax - ymin) * 0.75
		var sum := Vector3.ZERO
		var cnt := 0
		for v in _head_verts:
			if v.y >= ycut:
				sum += v
				cnt += 1
		if cnt > 0:
			_crown = sum / float(cnt)
	print("[HairSystem] 顶点哈希：%d 个模型顶点（格 %.0fmm），头模 %d 点，头顶中心 y=%.3f" % [
		_vtx_pos.size(), cell * 1000.0, _head_verts.size(), _crown.y])

# 在半径内取最近的 K 个模型顶点（写入 out）—— 只用邻近 27 个格子
func _nearest_verts(pt: Vector3, k: int, out: PackedInt32Array) -> void:
	# 不做排序：只取邻近格子里"半径内"的前 K 个顶点。
	# 碰撞只需要"附近有哪些面"的代表点，不需要精确第 K 近 —— 这样是 O(K) 而不是 O(K²)。
	out.clear()
	if not _vtx_ok:
		return
	var cell: float = vtx_cell
	var r2: float = vtx_query_radius * vtx_query_radius
	var gx := int(floor(pt.x / cell))
	var gy := int(floor(pt.y / cell))
	var gz := int(floor(pt.z / cell))
	for dx in range(-1, 2):
		for dy in range(-1, 2):
			for dz in range(-1, 2):
				var key := (gx + dx) * 1000003 + (gy + dy) * 10007 + (gz + dz)
				if not _vtx_grid.has(key):
					continue
				for vi in (_vtx_grid[key] as PackedInt32Array):
					if pt.distance_squared_to(_vtx_pos[vi]) < r2:
						out.append(vi)
						if out.size() >= k:
							return

# 把点推离模型表面：取缓存顶点里"最深"的那个，沿其法线推出去。
# 只在 margin 之外才算"进去了"；过远的顶点（>8cm）忽略，避免被远处的法线带偏。
# ---------------------------------------------------------------------------
# 惰性 SDF：按格子缓存有符号距离。查询 = 一次字典查找（O(1)）。
# sd > 0 在体外，sd < 0 在体内；法线保证外向（见 _build_vertex_hash 的统一处理）。
# ---------------------------------------------------------------------------
func _sdf_at(p: Vector3) -> Array:
	if not _vtx_ok:
		return [1.0, Vector3.UP]
	var gx := int(floor(p.x / _sdf_cell))
	var gy := int(floor(p.y / _sdf_cell))
	var gz := int(floor(p.z / _sdf_cell))
	var key := gx * 1000003 + gy * 10007 + gz
	# 缓存"该格的最近顶点下标"而不是距离：同一格里不同点各自现算有符号距离，
	# 精度只受顶点间距（~1cm）限制，不受格子大小限制。
	# （缓存距离会有格子级量化误差，实测 980 关节里 170 个漏挡。）
	var vi: int = _sdf_cv.get(key, -2)
	if vi == -2:
		vi = _nearest_vertex(p)
		_sdf_cv[key] = vi
	if vi < 0:
		return [1.0, Vector3.UP, 1.0]
	var dv := p - _vtx_pos[vi]
	var n := _vtx_nrm[vi]
	# 返回三件套：[有符号距离, 法线, 到该顶点的实际距离]。
	# 第三个是关键：关节深陷时找到的顶点在几厘米外，dot(p-v,n) 会因切向
	# 偏移而数值很小 ⇒"看着不深"⇒ 逃逸方向永远不触发（实测 94mm 深陷卡死）。
	return [dv.dot(n), n, dv.length()]

# 单点碰撞约束：sd < margin（在体内或贴得太近）就沿外法线推出去。
# 只沿法线移动，不动切向 —— 这是它与"瞬移到表面"的区别，避免抖动。
# 射线奇偶内外判定（用三角形哈希，只对少数"模糊关节"调用）。
# 有符号距离的盲区：关节深陷时最近顶点在几厘米外，切向偏移会让 dot 为正
# ⇒"看着在体外"⇒ 永远不推（实测 9cm 深陷关节的 sd 是正的）。
var _parity_d := Vector3(1.0, 0.371, 0.193).normalized()

func _ray_parity_inside(p: Vector3) -> bool:
	if not _tri_ok:
		return false
	var cnt := 0
	var cell: float = tri_cell
	var gx := int(floor(p.x / cell))
	var gy := int(floor(p.y / cell))
	var gz := int(floor(p.z / cell))
	for dx in range(-2, 3):
		for dy in range(-2, 3):
			for dz in range(-2, 3):
				var key := (gx + dx) * 1000003 + (gy + dy) * 10007 + (gz + dz)
				if not _tri_grid.has(key):
					continue
				for ti in (_tri_grid[key] as PackedInt32Array):
					var a: Vector3 = _tri_a[ti * 3]
					var e1: Vector3 = _tri_a[ti * 3 + 1] - a
					var e2: Vector3 = _tri_a[ti * 3 + 2] - a
					var h := _parity_d.cross(e2)
					var det := e1.dot(h)
					if absf(det) < 1e-12:
						continue
					var inv := 1.0 / det
					var s := p - a
					var u := s.dot(h) * inv
					if u < 0.0 or u > 1.0:
						continue
					var q := s.cross(e1)
					var vv := _parity_d.dot(q) * inv
					if vv < 0.0 or u + vv > 1.0:
						continue
					if e2.dot(q) * inv > 1e-6:
						cnt += 1
	return (cnt % 2) == 1

# 返回"应该施加的推挤位移"（不直接改点），便于调用方做限幅/邻域分摊
func _project_delta(p: Vector3, r: Array, margin: float) -> Vector3:
	var sd: float = r[0]
	var n: Vector3 = r[1]
	if r[2] > 0.025:
		if not _vtx_bounds.has_point(p):
			return Vector3.ZERO
		var pkey := int(floor(p.x / _sdf_cell)) * 1000003 + int(floor(p.y / _sdf_cell)) * 10007 + int(floor(p.z / _sdf_cell))
		var pin: Variant = _parity_cell.get(pkey, null)
		if pin == null:
			pin = _ray_parity_inside(p)
			_parity_cell[pkey] = pin
		if not pin:
			return Vector3.ZERO
		var ref := _head_center if p.y > _head_center.y else _body_center
		var esc := p - ref
		if esc.length_squared() > 1e-8:
			n = esc.normalized()
		sd = -r[2]
	elif sd >= margin:
		return Vector3.ZERO
	elif sd < -0.012:
		var ref2 := _head_center if p.y > _head_center.y else _body_center
		var esc2 := p - ref2
		if esc2.length_squared() > 1e-8:
			n = esc2.normalized()
	var delta := n * (margin - sd)
	var dl := delta.length()
	if dl > _max_push:
		delta = delta * (_max_push / dl)
	return delta

func _project_out(p: Vector3, margin: float) -> Vector3:
	var r: Array = _sdf_at(p)
	var sd: float = r[0]
	var n: Vector3 = r[1]
	if r[2] > 0.025:
		# 离任何顶点都远：要么悬空（多数），要么深陷（sd 因切向偏移失真）。
		# ① 包围盒外必在体外（悬空发梢走这条零成本出口）
		if not _vtx_bounds.has_point(p):
			return p
		# ② 盒内才做射线奇偶；结果按 10mm 格缓存（身体静止，永久有效）
		var pkey := int(floor(p.x / _sdf_cell)) * 1000003 + int(floor(p.y / _sdf_cell)) * 10007 + int(floor(p.z / _sdf_cell))
		var pin: Variant = _parity_cell.get(pkey, null)
		if pin == null:
			pin = _ray_parity_inside(p)
			_parity_cell[pkey] = pin
		if not pin:
			return p
		var ref := _head_center if p.y > _head_center.y else _body_center
		var esc := p - ref
		if esc.length_squared() > 1e-8:
			n = esc.normalized()
		sd = -r[2]   # 按深陷处理，推挤量按实际距离给
	elif sd >= margin:
		return p
	elif sd < -0.012:
		var ref2 := _head_center if p.y > _head_center.y else _body_center
		var esc2 := p - ref2
		if esc2.length_squared() > 1e-8:
			n = esc2.normalized()
	# 单次推挤限幅：深陷关节分多帧滑出，而不是一次瞬移 9cm
	# （瞬移会被 FTL 再分配给邻居，形成"深陷↔猛弹"的泵动 = 抖动尖峰）。
	# 限幅后每帧最多退 25mm，7 趟投影合计仍能追上快速转头。
	var delta := n * (margin - sd)
	var dl := delta.length()
	if dl > _max_push:
		delta = delta * (_max_push / dl)
	return p + delta

# XPBD 碰撞约束：对所有导引链的自由关节统一投影。
# 它只是统一循环里的**一个约束**，与链长/弯曲约束共同收敛 ——
# 不再是旧实现里"先链长后碰撞"的分相位执行（那会让两者互相打架形成抖动极限环）。
func _solve_collision_xpbd(final: bool = false) -> void:
	if not _vtx_ok or not collision_enabled:
		return
	var cap: float = _max_push_final if final else _max_push
	for c in _guides:
		var ch: Dictionary = _chains[c]
		var p: PackedVector3Array = ch["p"]
		var pv: PackedVector3Array = ch["prev"]
		var na: int = _attach[c] if c < _attach.size() else 1
		for j in range(na, p.size()):
			var old := p[j]
			var np := old
			var r: Array = _sdf_at(old)
			var d := _project_delta(old, r, contact_margin)
			if d.length_squared() > 1e-12:
				if d.length() > cap:
					d = d * (cap / d.length())
				np = old + d
			if np == old:
				continue
			var applied := np - old
			# 推挤按**邻域分摊**：一个关节被推出时，相邻关节一起走同样的量。
			# 三点整体平移不改变它们之间的段长 ⇒ 从构造上就不产生拉伸。
			# 只推单点会让相邻两段被拉长（实测达 250%），这就是"链条有弹性"的来源。
			p[j] = np
			if j > na:
				p[j - 1] = p[j - 1] + applied * neighbour_share
			if j + 1 < p.size():
				p[j + 1] = p[j + 1] + applied * neighbour_share
			# 投影量同步加到 prev ⇒ 不注入速度（否则形成抖动极限环）
			pv[j] = pv[j] + applied
		ch["p"] = p
		ch["prev"] = pv

func _push_out_verts_at(pt: Vector3, jv: PackedInt32Array, base: int, margin: float) -> Vector3:
	# 取缓存顶点里"最深"的那个，沿其法线推出去（有符号距离，比椭球精确）
	var best_d := -1e9
	var best_n := Vector3.ZERO
	for t in vtx_query_k:
		var idx := base + t
		if idx >= jv.size():
			break
		var vi: int = jv[idx]
		var d: float = (pt - _vtx_pos[vi]).dot(_vtx_nrm[vi])
		if d > best_d:
			best_d = d
			best_n = _vtx_nrm[vi]
	# best_d 是**有符号**距离（正=在体内，负=在体外）。
	# 只有当"在体外不足 margin"或"在体内"时才推：即 best_d > -margin。
	# 之前写成 best_d > -0.09 —— 于是体外 9cm 的点也会被再推 9cm，整头发被推散。
	if best_d > -margin:
		return pt + best_n * (margin - best_d)
	return pt

# 头皮附着点：在"脑壳上半部"的模型顶点里找离参考点最近的那个，
# 再沿法线外推一点。这样每条链的根落在**它自己那一撮头发的发际**上，
# 而不是所有链共用头顶那一个点。

# =====================================================================
# 三角形哈希 + 最近三角形查询（真正的网格 SDF）
# ---------------------------------------------------------------------
# 上一版用"邻近顶点 + 顶点法线"取最大有符号距离 —— 那不是 SDF：
# K 个顶点里混着方向无关的，max 会把它们挑出来当推挤方向，链条被推歪（漂移 100~160mm）。
#
# 正确做法（本段实现）：
#   ① body+head 三角形装进 5cm 均匀网格；
#   ② 查询 = 邻近 27 格里求"点到三角形的最短点"（Ericson 重心坐标算法）；
#   ③ 有符号距离 sd = dot(p - 最短点, 面法线)，sd < margin 就把点放到 最短点 + 法线·margin。
# 一次公式同时覆盖"在体内"和"贴在表面"两种情况，方向永远是最短点到面的法线，稳定。
# =====================================================================
var _tri_a := PackedVector3Array()      # 每三角形 3 个顶点，展开存储（9 float/三角形）
var _tri_n := PackedVector3Array()      # 面法线
var _tri_c := PackedVector3Array()      # 重心（候选粗筛用，避免扫描时算最短点）
var _tri_grid := {}
var _tri_ok := false
var _joint_tri: Array = []          # 每条链：每关节 tri_query_k 个候选三角形下标
var _joint_tri_ref: Array = []      # 每条链：每关节上次查询位置（判断是否需要重查）

func _build_tri_hash(parts: Dictionary) -> void:
	_tri_a.clear()
	_tri_n.clear()
	_tri_c.clear()
	_tri_grid.clear()
	_tri_ok = false
	var cell: float = tri_cell
	for key2 in ["body", "head"]:
		var mi: MeshInstance3D = parts.get(key2)
		if mi == null or mi.skin == null:
			continue
		var skin: Skin = mi.skin
		var nb: int = mini(_sk.get_bone_count(), skin.get_bind_count())
		var mats := []
		mats.resize(nb)
		for b in nb:
			mats[b] = _sk.get_bone_global_rest(b) * skin.get_bind_pose(b)
		for surf in mi.mesh.get_surface_count():
			var arr: Array = mi.mesh.surface_get_arrays(surf)
			var v: PackedVector3Array = arr[Mesh.ARRAY_VERTEX]
			var idx: PackedInt32Array = arr[Mesh.ARRAY_INDEX]
			var bo: PackedInt32Array = arr[Mesh.ARRAY_BONES]
			var we: PackedFloat32Array = arr[Mesh.ARRAY_WEIGHTS]
			var wp := PackedVector3Array()
			wp.resize(v.size())
			for i in v.size():
				var acc := Vector3.ZERO
				var ws := 0.0
				for k in 4:
					var w: float = we[i * 4 + k]
					if w <= 0.0:
						continue
					var bi: int = bo[i * 4 + k]
					if bi >= nb:
						continue
					acc += (mats[bi] as Transform3D) * v[i] * w
					ws += w
				wp[i] = acc / ws if ws > 0.0 else v[i]
			var t := 0
			while t + 2 < idx.size():
				var p0: Vector3 = wp[idx[t]]
				var p1: Vector3 = wp[idx[t + 1]]
				var p2: Vector3 = wp[idx[t + 2]]
				t += 3
				# 只保留头发可能触及的区域
				if maxf(p0.y, maxf(p1.y, p2.y)) < 0.95:
					continue
				if maxf(absf(p0.x), maxf(absf(p1.x), absf(p2.x))) > 0.46:
					continue
				if maxf(absf(p0.z), maxf(absf(p1.z), absf(p2.z))) > 0.37:
					continue
				var n := (p1 - p0).cross(p2 - p0)
				if n.length_squared() < 1e-12:
					continue
				var ti := _tri_a.size() / 3
				_tri_a.append(p0)
				_tri_a.append(p1)
				_tri_a.append(p2)
				_tri_n.append(n.normalized())
				_tri_c.append((p0 + p1 + p2) / 3.0)
				var mn: Vector3 = p0.min(p1).min(p2)
				var mx: Vector3 = p0.max(p1).max(p2)
				for gx in range(int(floor(mn.x / cell)), int(floor(mx.x / cell)) + 1):
					for gy in range(int(floor(mn.y / cell)), int(floor(mx.y / cell)) + 1):
						for gz in range(int(floor(mn.z / cell)), int(floor(mx.z / cell)) + 1):
							var key := gx * 1000003 + gy * 10007 + gz
							if not _tri_grid.has(key):
								_tri_grid[key] = PackedInt32Array()
							var bkt: PackedInt32Array = _tri_grid[key]
							bkt.append(ti)
							_tri_grid[key] = bkt
	_tri_ok = _tri_a.size() > 0
	print("[HairSystem] 三角形哈希：%d 个三角形（格 %.0fmm）" % [_tri_a.size() / 3, cell * 1000.0])

# 点到三角形的最短点（Ericson, Real-Time Collision Detection）
func _closest_on_tri(p: Vector3, ti: int) -> Vector3:
	var a := _tri_a[ti * 3]
	var b := _tri_a[ti * 3 + 1]
	var c := _tri_a[ti * 3 + 2]
	var ab := b - a
	var ac := c - a
	var ap := p - a
	var d1 := ab.dot(ap)
	var d2 := ac.dot(ap)
	if d1 <= 0.0 and d2 <= 0.0:
		return a
	var bp := p - b
	var d3 := ab.dot(bp)
	var d4 := ac.dot(bp)
	if d3 >= 0.0 and d4 <= d3:
		return b
	var vc := d1 * d4 - d3 * d2
	if vc <= 0.0 and d1 >= 0.0 and d3 <= 0.0:
		return a + ab * (d1 / (d1 - d3))
	var cp := p - c
	var d5 := ab.dot(cp)
	var d6 := ac.dot(cp)
	if d6 >= 0.0 and d5 <= d6:
		return c
	var vb := d5 * d2 - d1 * d6
	if vb <= 0.0 and d2 >= 0.0 and d6 <= 0.0:
		return a + ac * (d2 / (d2 - d6))
	var va := d3 * d6 - d5 * d4
	if va <= 0.0 and (d4 - d3) >= 0.0 and (d5 - d6) >= 0.0:
		return b + (c - b) * ((d4 - d3) / ((d4 - d3) + (d5 - d6)))
	var den := 1.0 / (va + vb + vc)
	return a + ab * (vb * den) + ac * (vc * den)

# 求最近三角形：返回 [最短点, 面法线, 有符号距离]（正=体外）
func _closest_tri(p: Vector3) -> Array:
	var cell: float = tri_cell
	var gx := int(floor(p.x / cell))
	var gy := int(floor(p.y / cell))
	var gz := int(floor(p.z / cell))
	var best := 1e18
	var bc := p
	var bn := Vector3.UP
	for dx in range(-1, 2):
		for dy in range(-1, 2):
			for dz in range(-1, 2):
				var key := (gx + dx) * 1000003 + (gy + dy) * 10007 + (gz + dz)
				if not _tri_grid.has(key):
					continue
				for ti in (_tri_grid[key] as PackedInt32Array):
					var cp := _closest_on_tri(p, ti)
					var d := p.distance_squared_to(cp)
					if d < best:
						best = d
						bc = cp
						bn = _tri_n[ti]
	if best > 1e17:
		return [p, Vector3.UP, 1.0]
	var dl := p - bc
	var dist := sqrt(best)
	var sd := dl.dot(bn)
	return [bc, bn, dist if sd >= 0.0 else -dist]


# ---------------------------------------------------------------------
# 候选三角形缓存：查询很贵（邻近 27 格 ~200 个三角形 × 最短点计算），
# 所以每个关节缓存 K 个最近三角形，只有"关节移动超过阈值"时才重查。
# 静止/慢速时几乎不重查，快速摆动时自动跟上。
# ---------------------------------------------------------------------
func _nearest_tris(p: Vector3, k: int, out: PackedInt32Array) -> void:
	out.clear()
	if not _tri_ok:
		return
	var cell: float = tri_cell
	var gx := int(floor(p.x / cell))
	var gy := int(floor(p.y / cell))
	var gz := int(floor(p.z / cell))
	# 不做排序：取邻近格子里"半径内"的前 K 个三角形。
	# 真正的最近三角形由 _push_out_tris_cached 在候选里挑 ——
	# 配合椭球粗挡 + 逐帧迭代已经足够，排序反而让刷新变慢 20 倍。
	var r2: float = tri_query_radius * tri_query_radius
	for dx in range(-1, 2):
		for dy in range(-1, 2):
			for dz in range(-1, 2):
				var key := (gx + dx) * 1000003 + (gy + dy) * 10007 + (gz + dz)
				if not _tri_grid.has(key):
					continue
				for ti in (_tri_grid[key] as PackedInt32Array):
					# 用重心当粗筛（便宜），最短点计算留给 _push_out_tris_cached
					if p.distance_squared_to(_tri_c[ti]) < r2:
						out.append(ti)
						if out.size() >= k:
							return

# 按需刷新：关节没怎么动就用旧候选，省掉重查
func _refresh_joint_tri() -> void:
	if not _tri_ok or _chains.is_empty():
		return
	while _joint_tri.size() < _chains.size():
		_joint_tri.append(PackedInt32Array())
		_joint_tri_ref.append(PackedVector3Array())
	var tmp := PackedInt32Array()
	# 插值链也要有候选（它们驱动大部分可见头发，不推就会陷进身体里）
	var list: Array = range(_chains.size()) if interpolated_collision else _guides.duplicate()
	for ci in list:
		if ci >= _chains.size():
			continue
		var p: PackedVector3Array = _chains[ci]["p"]
		var n_j: int = p.size()
		var cur: PackedInt32Array = _joint_tri[ci]
		var ref: PackedVector3Array = _joint_tri_ref[ci]
		# 只按"参考位置数组是否够长"判断有效性。
		# 之前写成 cur.size() < n_j * tri_query_k —— 半径内不足 K 个时永远为真，
		# 于是每帧全量重查（实测 81ms/帧）。
		var need: bool = ref.size() < n_j
		if not need:
			for j in range(_attach[ci] if ci < _attach.size() else 1, n_j):
				if p[j].distance_squared_to(ref[j]) > tri_refresh_dist * tri_refresh_dist:
					need = true
					break
		if not need:
			continue
		var acc := PackedInt32Array()
		var r2 := PackedVector3Array()
		for j in n_j:
			_nearest_tris(p[j], tri_query_k, tmp)
			for vi in tmp:
				acc.append(vi)
			r2.append(p[j])
		_joint_tri[ci] = acc
		_joint_tri_ref[ci] = r2
# 把点放到"最近表面 + margin"：在体内、贴表面，一次公式统一处理
func _push_out_tris_cached(p: Vector3, cached: PackedInt32Array, base: int, margin: float) -> Vector3:
	if not _tri_ok:
		return p
	var best := 1e18
	var bc := p
	var bn := Vector3.UP
	for t in tri_query_k:
		var idx := base + t
		if idx >= cached.size():
			break
		var ti: int = cached[idx]
		var cp := _closest_on_tri(p, ti)
		var d := p.distance_squared_to(cp)
		if d < best:
			best = d
			bc = cp
			bn = _tri_n[ti]
	if best > 1e17:
		return p
	var dist := sqrt(best)
	var sd := (p - bc).dot(bn)
	sd = dist if sd >= 0.0 else -dist
	if sd >= margin:
		return p
	return bc + bn * margin

# 缓存粗判：返回相对最近缓存三角形的有符号距离（可能偏大，因为缓存未必含真正的最近面）
func _sd_from_cached(p: Vector3, cached: PackedInt32Array, base: int) -> float:
	var best := 1e18
	var bc := p
	var bn := Vector3.UP
	for t in tri_query_k:
		var idx := base + t
		if idx >= cached.size():
			break
		var ti: int = cached[idx]
		var cp := _closest_on_tri(p, ti)
		var d := p.distance_squared_to(cp)
		if d < best:
			best = d
			bc = cp
			bn = _tri_n[ti]
	if best > 1e17:
		return 1e9
	var dist := sqrt(best)
	return dist if (p - bc).dot(bn) >= 0.0 else -dist

# 硬约束版：先粗判，只有"疑似穿模/贴面"时才做**全量精确查询**。
# 全量查询扫邻近 27 格（5cm 格 ⇒ 覆盖 15cm），对毫米级 margin 的判定是精确的；
# 之所以必须这样，是因为缓存里的 K 个三角形未必含真正的最近面 ——
# 只按缓存推会出现"看起来推了、其实还在里面"（实测残留 28 个关节、最深 72mm）。
func _push_out_tris_hard(p: Vector3, cached: PackedInt32Array, base: int, margin: float) -> Vector3:
	if not _tri_ok:
		return p
	if _sd_from_cached(p, cached, base) >= margin + 0.02:
		return p                      # 明显在体外，零额外开销
	var r: Array = _closest_tri(p)
	# 方向用**顶点法线**，不用面法线：
	# 面法线由三角形绕向决定，而这套资产的绕向并不一致 ——
	# 用它会把关节往身体里面推（实测反而从 28 个恶化到 266 个在体内）。
	# ARRAY_NORMAL 是美术烘好的、一致可靠的。
	var cp2: Vector3 = r[0]
	var nrm := _normal_at(cp2)
	if (p - cp2).dot(nrm) >= margin:
		return p
	return cp2 + nrm * margin

# 最近顶点的法线（ARRAY_NORMAL，可靠）
func _normal_at(p: Vector3) -> Vector3:
	if not _vtx_ok:
		return Vector3.UP
	var cell: float = vtx_cell
	var gx := int(floor(p.x / cell))
	var gy := int(floor(p.y / cell))
	var gz := int(floor(p.z / cell))
	var best := 1e18
	var bn := Vector3.UP
	for dx in range(-1, 2):
		for dy in range(-1, 2):
			for dz in range(-1, 2):
				var key := (gx + dx) * 1000003 + (gy + dy) * 10007 + (gz + dz)
				if not _vtx_grid.has(key):
					continue
				for vi in (_vtx_grid[key] as PackedInt32Array):
					var d: float = p.distance_squared_to(_vtx_pos[vi])
					if d < best:
						best = d
						bn = _vtx_nrm[vi]
	return bn


# =====================================================================
# 实时「链条节点 ↔ mesh 顶点」距离接触（逐帧计算）
# ---------------------------------------------------------------------
# 用顶点哈希（11879 个顶点 / 30mm 格）找最近顶点，沿**顶点法线**做有符号距离：
#     sd = dot(p - v, n_v)        （正 = 在体内）
#     sd < margin  ⇒  p = v + n_v * margin
# 为什么不直接用三角形：三角形查询每个关节要遍历邻近 ~200 个三角形（>40ms/帧）；
# 顶点查询只扫 27 格、每格约 4 个顶点 ≈ 100 次距离比较，便宜约 20 倍。
# 顶点间距约 1cm，对毫米级接触间隙足够。
# 顶点法线来自 ARRAY_NORMAL，与三角形绕向无关 —— 这套资产绕向不一致，
# 面法线会把点往里推（实测从 28 个恶化到 266 个）。
# =====================================================================
func _nearest_vertex(p: Vector3) -> int:
	if not _vtx_ok:
		return -1
	# 渐进扩圈：1、2、4、8 格（30mm 格 ⇒ 最远 ~24cm）。
	# 只扫固定 27 格时，陷在胸腔/颅腔中部的关节周围 6cm 内没有顶点，
	# 会返回 -1 ⇒ SDF 把它当"体外" ⇒ 深陷关节对碰撞**隐身**（实测 89mm 穿透）。
	var cell: float = vtx_cell
	var gx := int(floor(p.x / cell))
	var gy := int(floor(p.y / cell))
	var gz := int(floor(p.z / cell))
	var best := 1e18
	var bi := -1
	for r in [1, 2, 4, 8]:
		var best2 := 1e18
		var bi2 := -1
		for dx in range(-r, r + 1):
			for dy in range(-r, r + 1):
				for dz in range(-r, r + 1):
					if r > 1 and (absi(dx) < r and absi(dy) < r and absi(dz) < r):
						continue   # 内圈已扫过
					var key := (gx + dx) * 1000003 + (gy + dy) * 10007 + (gz + dz)
					if not _vtx_grid.has(key):
						continue
					for vi in (_vtx_grid[key] as PackedInt32Array):
						var d: float = p.distance_squared_to(_vtx_pos[vi])
						if d < best2:
							best2 = d
							bi2 = vi
		if bi2 >= 0:
			best = best2
			bi = bi2
			if r >= 2:
				break   # 外圈命中即可：说明内圈确实没有 mesh
	return bi

# 沿最近顶点法线把点放到 margin 之外。relax < 1 时分步推进 ——
# 一次推到位会和"不伸缩"约束打架，产生逐帧来回抖动的极限环。
func _vert_contact(p: Vector3, margin: float, relax: float) -> Vector3:
	var vi := _nearest_vertex(p)
	if vi < 0:
		return p
	var v := _vtx_pos[vi]
	var n := _vtx_nrm[vi]
	var sd := (p - v).dot(n)
	if sd >= margin:
		return p
	var target := v + n * margin
	return p.lerp(target, relax)
# 直接查询（不缓存）：用于静止预推等一次性场合
func _push_out_tris(p: Vector3, margin: float) -> Vector3:
	if not _tri_ok:
		return p
	var r: Array = _closest_tri(p)
	var sd: float = r[2]
	if sd >= margin:
		return p
	return (r[0] as Vector3) + (r[1] as Vector3) * margin
func _scalp_root(ref: Vector3) -> Vector3:
	# 之前写成"在 y > 头心+2cm 的点里找离 ref 最近的" —— 搜索半径 20cm，
	# 于是前发的参考点会匹配到**额头甚至下巴**，头发就长到脸上了。
	#
	# 正确做法：把"该撮头发的方向"从头顶中心投到**脑壳上半球**上。
	#   d = ref - 头顶中心（强制 y 分量不为负，保证落在颅顶而不是颅底）
	#   在头顶以上的头模顶点里，取与 d 方向最一致的那个 ⇒ 必然是发际线上的点。
	if _head_verts.size() == 0:
		return ref
	var d := ref - _crown
	var dl := maxf(d.length(), 1e-4)
	d.y = maxf(d.y, 0.30 * dl)          # 不许朝下 —— 否则会投到下巴
	d = d.normalized()
	var best := -2.0
	var bp := _crown
	for v in _head_verts:
		if v.y < _crown.y - 0.02:
			continue                     # 只考虑颅顶以上
		var u := v - _crown
		var ul := u.length()
		if ul < 1e-5:
			continue
		var s := (u / ul).dot(d)
		if s > best:
			best = s
			bp = v
	# 沿"从头顶中心指向该点"的方向再往外推一点，避免根埋在头皮里
	var bn := (bp - _crown)
	if bn.length_squared() > 1e-8:
		bn = bn.normalized()
	else:
		bn = Vector3.UP
	return bp + bn * 0.008

# 每条链、每个关节缓存最近的 K 个模型顶点（碰撞热路径只遍历这些）
func _refresh_joint_vtx() -> void:
	_joint_vtx.clear()
	if not _vtx_ok:
		return
	for ci in _chains.size():
		var p: PackedVector3Array = _chains[ci]["p"]
		var acc := PackedInt32Array()
		var tmp := PackedInt32Array()
		for j in p.size():
			_nearest_verts(p[j], vtx_query_k, tmp)
			for vi in tmp:
				acc.append(vi)
		_joint_vtx.append(acc)
# =====================================================================
# 网状发根排布：在头皮上按 (行,列) 网格生成发根，每根一条物理链。
# 旧方案的所有导引链共用 1~2 个头顶骨组质心 ⇒ 根全挤在头顶。
# 新方案：头模顶点 → 球坐标 (极角 φ, 方位角 θ) 网格，每个格点取方向最一致的
# 头模顶点作发根，沿该处法线向外垂出链条 —— 根部天然呈**网状**覆盖整个头皮。
# 面部区（+Z 中低段）不长头发，直接跳过。
# =====================================================================
func _build_scalp_grid_chains() -> Array:
	var out := []
	if _head_verts.is_empty():
		return out
	var hc := Vector3.ZERO
	for v in _head_verts:
		hc += v
	hc = hc / float(_head_verts.size())
	var rows: int = maxi(grid_rows, 2)
	var cols: int = maxi(grid_cols, 3)
	var n: int = maxi(joints_per_chain, 4)
	for r in rows:
		var phi: float = lerpf(0.30, 1.60, (float(r) + 0.5) / float(rows))
		var L: float = lerpf(scalp_len_top, scalp_len_bottom, float(r) / float(maxi(rows - 1, 1)))
		for c in cols:
			var theta: float = TAU * (float(c) + 0.5) / float(cols)
			var dir := Vector3(sin(phi) * sin(theta), cos(phi), sin(phi) * cos(theta)).normalized()
			if dir.z > 0.60 and phi > 0.95:
				continue
			var best := -2.0
			var bp := Vector3.ZERO
			for v in _head_verts:
				var u := v - hc
				var ul := u.length()
				if ul < 1e-5:
					continue
				var s := (u / ul).dot(dir)
				if s > best:
					best = s
					bp = v
			if best < 0.40:
				continue
			var nrm := (bp - hc).normalized()
			var root := bp + nrm * 0.006
			var q := PackedVector3Array()
			q.resize(n)
			var seg := L / float(n - 1)
			var d0 := nrm
			q[0] = root
			for j in range(1, n):
				var t := float(j) / float(n - 1)
				var dd := d0.lerp(Vector3(0, -1, 0), clampf(t * 1.5, 0.0, 1.0)).normalized()
				q[j] = q[j - 1] + dd * seg
			var part := "hair_b" if dir.z < 0.0 else "hair_f"
			out.append({"part": part, "poly": q})
	return out

func _build_interpolated(n: int) -> void:
	# 按 chain_target 精确分配插值链数量（缺口数 5：前发 3 + 后发 2）
	var want: int = maxi(chain_target - _chains.size(), 0)
	if want <= 0:
		return
	var gaps := 0
	var byp := {}
	for gi in _guides:
		var c0: Dictionary = _chains[gi]
		if not byp.has(c0["part"]):
			byp[c0["part"]] = []
		byp[c0["part"]].append(gi)
	for pt in byp.keys():
		gaps += maxi((byp[pt] as Array).size() - 1, 0)
	if gaps <= 0:
		return
	var per := []
	var base: int = want / gaps
	var extra: int = want % gaps
	for i in gaps:
		per.append(base + (1 if i < extra else 0))
	# 按部位分组，组内按「链中段 x」排序 → 左右相邻关系
	var by_part := {}
	for gi in _guides:
		var c: Dictionary = _chains[gi]
		if not by_part.has(c["part"]):
			by_part[c["part"]] = []
		by_part[c["part"]].append(gi)
	var gap_i := 0
	for part in by_part.keys():
		var ids: Array = by_part[part]
		ids.sort_custom(func(a, b): return _chain_mid_x(_chains[a]) < _chain_mid_x(_chains[b]))
		for i in ids.size() - 1:
			var k: int = per[gap_i] if gap_i < per.size() else 0
			gap_i += 1
			if k <= 0:
				continue
			var ia: int = ids[i]
			var ib: int = ids[i + 1]
			var qa: PackedVector3Array = _chains[ia]["q"]
			var qb: PackedVector3Array = _chains[ib]["q"]
			for t in range(1, k + 1):
				var f := float(t) / float(k + 1)
				var qv := PackedVector3Array()
				for j in n:
					qv.append(qa[j].lerp(qb[j], f))
				_chains.append(_make_chain(part, qv, false, ia, ib, f))

func _chain_mid_x(c: Dictionary) -> float:
	var q: PackedVector3Array = c["q"]
	return q[int(q.size() * 0.5)].x

# ---------------- 几何：顶点群质心 ----------------
func _collect_hair_groups() -> Dictionary:
	var groups := {}
	for mi in _hair_meshes:
		var nb := _sk.get_bone_count()
		for surf in mi.mesh.get_surface_count():
			var arrays: Array = mi.mesh.surface_get_arrays(surf)
			if arrays.size() <= Mesh.ARRAY_WEIGHTS or arrays[Mesh.ARRAY_WEIGHTS] == null:
				continue
			var verts: PackedVector3Array = arrays[Mesh.ARRAY_VERTEX]
			var bones: PackedInt32Array = arrays[Mesh.ARRAY_BONES]
			var weights: PackedFloat32Array = arrays[Mesh.ARRAY_WEIGHTS]
			for i in verts.size():
				var best := -1
				var bw := 0.0
				for k in 4:
					var w: float = weights[i * 4 + k]
					if w > bw:
						bw = w; best = bones[i * 4 + k]
				if best < 0 or best >= nb:
					continue
				var nm := _sk.get_bone_name(best)
				if not nm.begins_with("cf_J_hair") and not nm.begins_with("cf_N_J_hair"):
					continue
				if not groups.has(nm):
					groups[nm] = []
				groups[nm].append(_A * verts[i])
	return groups

# 由「骨 → 顶点群质心」拼出链折线；缺顶点的骨用相邻关节插值/外推补上
func _chain_polyline(bone_names: Array, groups: Dictionary) -> PackedVector3Array:
	var pts := []
	var known := []
	var root_max_y := -INF
	if groups.has(bone_names[0]):
		for pt in groups[bone_names[0]]:
			root_max_y = maxf(root_max_y, (pt as Vector3).y)
	for i in bone_names.size():
		var nm: String = bone_names[i]
		var v := Vector3.ZERO
		var ok := false
		if groups.has(nm) and not groups[nm].is_empty():
			var arr: Array = groups[nm]
			if i == 0:
				# 根关节：只取该骨顶点群里最靠头顶的那 5cm，得到「发际/头顶附着点」
				var cut := root_max_y - 0.05
				var s := Vector3.ZERO
				var cnt := 0
				for pt in arr:
					if (pt as Vector3).y >= cut:
						s += pt; cnt += 1
				if cnt == 0:
					s = arr[0] if arr.size() > 0 else Vector3.ZERO
					cnt = 1
				v = s / float(cnt)
			else:
				v = Vector3.ZERO
				for pt in arr:
					v += pt
				v /= float(arr.size())
			ok = true
		pts.append(v)
		known.append(ok)
	# 补齐未知关节
	var first_known := -1
	for i in known.size():
		if known[i]:
			first_known = i
			break
	if first_known < 0:
		return PackedVector3Array()
	# 末尾外推方向
	for i in known.size():
		if known[i]:
			continue
		var prev := i - 1
		while prev >= 0 and not known[prev]:
			prev -= 1
		var next := i + 1
		while next < known.size() and not known[next]:
			next += 1
		if prev >= 0 and next < known.size():
			var f := float(i - prev) / float(next - prev)
			pts[i] = (pts[prev] as Vector3).lerp(pts[next], f)
		elif prev >= 0:
			# 末尾：沿最后一个已知方向外推，步长 = 前一段
			var pp := prev - 1
			while pp >= 0 and not known[pp]:
				pp -= 1
			var dir := Vector3(0.0, -1.0, 0.0)
			if pp >= 0:
				dir = (pts[prev] as Vector3) - (pts[pp] as Vector3)
			var step: float = dir.length()
			if step < 1e-5:
				step = 0.03
			pts[i] = (pts[i - 1] as Vector3) + dir.normalized() * step
		else:
			return PackedVector3Array()
	var out := PackedVector3Array()
	for p in pts:
		out.append(p)
	return out

static func _resample(poly: PackedVector3Array, n: int) -> PackedVector3Array:
	var seg := PackedFloat32Array()
	var total := 0.0
	for i in poly.size() - 1:
		var l := poly[i].distance_to(poly[i + 1])
		seg.append(l); total += l
	var out := PackedVector3Array()
	if total < 1e-6:
		for i in n:
			out.append(poly[0])
		return out
	for k in n:
		var target := total * float(k) / float(n - 1)
		var acc := 0.0
		var j := 0
		while j < seg.size() - 1 and acc + seg[j] < target:
			acc += seg[j]; j += 1
		var t := 0.0 if seg[j] < 1e-9 else clampf((target - acc) / seg[j], 0.0, 1.0)
		out.append(poly[j].lerp(poly[j + 1], t))
	return out

# ---------------- 新建骨头 + 蒙皮 ----------------
func _create_bones_and_skin(_parts: Dictionary) -> void:
	var head_rest: Transform3D = _sk.get_bone_global_rest(_head)
	var ci := 0
	for c in _chains:
		var q: PackedVector3Array = c["q"]
		var arr := PackedInt32Array()
		for j in q.size():
			var b := _sk.add_bone("hs2_hair_%d_%d" % [ci, j])
			# 扁平挂到头骨下：全局 rest = 关节静止位姿（纯平移），bind = rest⁻¹·A
			_sk.set_bone_parent(b, _head)
			_sk.set_bone_rest(b, head_rest.affine_inverse() * Transform3D(Basis(), q[j]))
			_sk.reset_bone_pose(b)
			arr.append(b)
			_bones.append(b)
			_bone_rc[b] = Vector2i(ci, j)
		c["bones"] = arr
		ci += 1
	# 统一蒙皮（复制原 skin，追加新骨的 bind）
	var base_skin: Skin = null
	for mi in _hair_meshes:
		if mi.skin != null:
			base_skin = mi.skin
			break
	if base_skin == null:
		return
	var skin: Skin = base_skin.duplicate() as Skin
	var need := _sk.get_bone_count()
	while skin.get_bind_count() < need:
		skin.add_bind(skin.get_bind_count(), Transform3D())
	for b in _bones:
		var q := _joint_rest(b)
		var inv := Transform3D(Basis(), -q)
		skin.set_bind_pose(b, inv * _A)
	for mi in _hair_meshes:
		_reweight_mesh(mi, skin)

func _joint_rest(b: int) -> Vector3:
	return _sk.get_bone_global_rest(b).origin

func _reweight_mesh(mi: MeshInstance3D, skin: Skin) -> void:
	var polys := []
	for c in _chains:
		polys.append(c["q"])
	var new_w := []
	var tangents := PackedVector3Array()
	var strand_t := PackedFloat32Array()
	var inner_ao := PackedFloat32Array()
	var adj := {}
	var offsets := PackedInt32Array()
	var head_center: Vector3 = _sk.get_bone_global_rest(_head).origin
	# 头骨原点不在头里（见 README），用头部碰撞体中心当"头心"
	for cc in _colliders:
		if int(cc["bone"]) == _head:
			head_center = cc["rest_center"]
			break
	for surf in mi.mesh.get_surface_count():
		offsets.append(new_w.size())
		var arrays: Array = mi.mesh.surface_get_arrays(surf)
		var verts: PackedVector3Array = arrays[Mesh.ARRAY_VERTEX]
		var norms: PackedVector3Array = arrays[Mesh.ARRAY_NORMAL]
		for i in verts.size():
			var q: Vector3 = _A * verts[i]
			new_w.append(_weights_for(q, polys))
			# 烘焙发丝切线（沿发链方向，mesh 空间）+ 根梢参数 / 朝内遮蔽
			var si := _strand_info(q, polys)
			var t_p: Vector3 = _chain_tangent(polys[si["ci"]], si["seg"])
			var t_m: Vector3 = _A.basis.inverse() * t_p
			if t_m.length_squared() < 1e-9:
				t_m = Vector3(0.0, 1.0, 0.0)
			tangents.append(t_m.normalized())
			strand_t.append(si["t01"])
			var nrm: Vector3 = norms[i] if norms.size() > i else Vector3.ZERO
			var outward: Vector3 = (q - head_center)
			var inner := 0.0
			if nrm.length_squared() > 1e-9 and outward.length_squared() > 1e-9:
				inner = clampf(-nrm.normalized().dot(outward.normalized()), 0.0, 1.0)
			inner_ao.append(inner)
		var idx: PackedInt32Array = arrays[Mesh.ARRAY_INDEX]
		var t := 0
		while t + 2 < idx.size():
			for e in 3:
				_add_edge(adj, offsets[surf] + idx[t + e], offsets[surf] + idx[t + (e + 1) % 3])
			t += 3
	# 权重在网格上做几轮平滑，避免同一张发片被拆到两条链上（撕裂）
	_smooth_weights(new_w, adj, 3)
	var src := mi.mesh
	var out := ArrayMesh.new()
	out.resource_name = src.resource_name
	for surf in src.get_surface_count():
		var arrays2: Array = src.surface_get_arrays(surf)
		var cnt: int = (arrays2[Mesh.ARRAY_VERTEX] as PackedVector3Array).size()
		var bones := PackedInt32Array(); bones.resize(cnt * 4)
		var wts := PackedFloat32Array(); wts.resize(cnt * 4)
		var off: int = offsets[surf]
		for i in cnt:
			var w: Array = new_w[off + i]
			for k in 4:
				if k < w.size():
					bones[i * 4 + k] = w[k][0]
					wts[i * 4 + k] = w[k][1]
		arrays2[Mesh.ARRAY_BONES] = bones
		arrays2[Mesh.ARRAY_WEIGHTS] = wts
		var tan := PackedFloat32Array(); tan.resize(cnt * 4)
		var col := PackedColorArray(); col.resize(cnt)
		for i in cnt:
			var tv: Vector3 = tangents[off + i]
			tan[i * 4 + 0] = tv.x
			tan[i * 4 + 1] = tv.y
			tan[i * 4 + 2] = tv.z
			tan[i * 4 + 3] = 1.0   # binormal 手性
			col[i] = Color(strand_t[off + i], inner_ao[off + i], 0.0, 1.0)
		arrays2[Mesh.ARRAY_TANGENT] = tan
		arrays2[Mesh.ARRAY_COLOR] = col
		out.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arrays2)
		out.surface_set_name(surf, src.surface_get_name(surf))
		out.surface_set_material(surf, src.surface_get_material(surf))
	mi.mesh = out
	mi.skin = skin
	mi.skeleton = NodePath("..")

# 顶点在发链上的位置：返回 {ci, seg, t01}（t01 = 沿整条链的归一化弧长参数）
func _strand_info(q: Vector3, polys: Array) -> Dictionary:
	var bd := INF
	var bc := 0
	var bs := 0
	var bt := 0.0
	for ci in polys.size():
		var r := _closest_on_poly(q, polys[ci])
		if r[0] < bd:
			bd = r[0]
			bc = ci
			bs = r[1]
			bt = r[2]
	var n: int = (polys[bc] as PackedVector3Array).size()
	return {"ci": bc, "seg": bs, "t01": clampf((float(bs) + bt) / float(maxi(n - 1, 1)), 0.0, 1.0)}

static func _chain_tangent(poly: PackedVector3Array, seg: int) -> Vector3:
	var a: Vector3 = poly[clampi(seg, 0, poly.size() - 2)]
	var b: Vector3 = poly[clampi(seg + 1, 1, poly.size() - 1)]
	var d := b - a
	return d.normalized() if d.length_squared() > 1e-12 else Vector3(0.0, -1.0, 0.0)

# 一个顶点的权重：取最近的若干条链（链条间插值），链内再按投影位置混合相邻两关节
func _weights_for(q: Vector3, polys: Array) -> Array:
	var hits := []
	for ci in polys.size():
		var r := _closest_on_poly(q, polys[ci])
		hits.append({"ci": ci, "d": r[0], "seg": r[1], "t": r[2]})
	hits.sort_custom(func(a, b): return a["d"] < b["d"])
	var acc := {}
	var total := 0.0
	var used := 0
	for h in hits:
		var d: float = h["d"]
		var wc := 1.0 / (d * d + 1e-6)
		# 只取足够近的链（第一条的 3 倍距离内），最多 3 条
		if used >= 3 or (used > 0 and d > hits[0]["d"] * 3.0 + 1e-4):
			break
		var c: Dictionary = _chains[h["ci"]]
		var seg: int = h["seg"]
		var t: float = h["t"]
		var bl: PackedInt32Array = c["bones"]
		acc[bl[seg]] = float(acc.get(bl[seg], 0.0)) + wc * (1.0 - t)
		acc[bl[seg + 1]] = float(acc.get(bl[seg + 1], 0.0)) + wc * t
		total += wc
		used += 1
	if total <= 0.0:
		return [[_chains[0]["bones"][0], 1.0]]
	var pairs := []
	for b in acc.keys():
		pairs.append([b, float(acc[b]) / total])
	pairs.sort_custom(func(a, b): return a[1] > b[1])
	while pairs.size() > 4:
		pairs.pop_back()
	var s := 0.0
	for p in pairs:
		s += p[1]
	for p in pairs:
		p[1] = p[1] / s
	return pairs

static func _add_edge(adj: Dictionary, a: int, b: int) -> void:
	if a == b:
		return
	if not adj.has(a):
		adj[a] = PackedInt32Array()
	var la: PackedInt32Array = adj[a]
	for v in la:
		if v == b:
			return
	la.append(b)
	adj[a] = la
	if not adj.has(b):
		adj[b] = PackedInt32Array()
	var lb: PackedInt32Array = adj[b]
	lb.append(a)
	adj[b] = lb

static func _smooth_weights(new_w: Array, adj: Dictionary, rounds: int) -> void:
	for r in rounds:
		var out := []
		out.resize(new_w.size())
		for i in new_w.size():
			var cur: Array = new_w[i]
			var nbs: PackedInt32Array = adj.get(i, PackedInt32Array())
			if nbs.is_empty():
				out[i] = cur
				continue
			var mix := {}
			for p in cur:
				mix[p[0]] = float(mix.get(p[0], 0.0)) + p[1] * 0.5
			var share := 0.5 / float(nbs.size())
			for nb in nbs:
				for p in new_w[nb]:
					mix[p[0]] = float(mix.get(p[0], 0.0)) + p[1] * share
			var pairs := []
			for b in mix.keys():
				pairs.append([b, mix[b]])
			pairs.sort_custom(func(a, b): return a[1] > b[1])
			while pairs.size() > 4:
				pairs.pop_back()
			var s := 0.0
			for p in pairs:
				s += p[1]
			if s > 0.0:
				for p in pairs:
					p[1] = p[1] / s
			out[i] = pairs
		for i in new_w.size():
			new_w[i] = out[i]

static func _closest_on_poly(q: Vector3, poly: PackedVector3Array) -> Array:
	var bd := INF
	var bs := 0
	var bt := 0.0
	for i in poly.size() - 1:
		var a := poly[i]
		var ab := poly[i + 1] - a
		var l2 := ab.length_squared()
		var t := 0.0 if l2 < 1e-12 else clampf((q - a).dot(ab) / l2, 0.0, 1.0)
		var d := q.distance_squared_to(a + ab * t)
		if d < bd:
			bd = d; bs = i; bt = t
	return [sqrt(bd), bs, bt]

# ---------------- 碰撞体 ----------------
# 碰撞体：**直接从「蒙皮后的 body+head 网格顶点」拟合**。
#
# 为什么推倒重来：
#   ① 之前头部用 global_rest(Head_s)·bind(Head_s) 把所有头模顶点变换过去 —— 但头模顶点
#      其实是绑在**面骨**上的，不是绑在 Head_s 上。于是算出来的椭球整整偏了 1.6m
#      （中心落在 y≈0，而头在 y≈1.66），等于**根本没有头部碰撞**。严格射线判据实测：
#      264 个链关节里有 173 个在 mesh 内部，最深 61.7mm —— 而椭球判据却报 0。
#   ② 固定常量表（脊柱那几颗椭球）也盖不住背/肩/胸/手臂的真实表面。
#
# 现在统一用「按蒙皮公式算出的真实顶点」拟合（在 rest 空间，运行时由骨 delta 带动）：
#   沿竖直方向分带 → 带内沿 x 聚类（把手臂和躯干分开）→
#   每颗椭球按该簇顶点的最大椭球范数放大 ⇒ **包含性可验证**（构建时打印未覆盖数）。
# 碰撞体 = 「实测常量表（躯干/胸/肩/臂）」 + 「按 pose 空间网格拟合的头部椭球堆」。
#
# 头部为什么必须拟合而不能用常量/ AABB：
#   ① 之前用 global_rest(Head_s)·bind(Head_s) 把所有头模顶点变换过去 —— 但头模顶点其实绑在
#      **面骨**上，不是绑在 Head_s 上。算出来的椭球整整偏了 1.6m（中心在 y≈0，头在 y≈1.66），
#      等于**根本没有头部碰撞**。严格射线判据实测：264 个链关节里 173 个在 mesh 内，最深 61.7mm，
#      而当时的椭球判据报 0 —— 这就是"链条穿过脑袋"的真正原因。
#   ② AABB 内接椭球是内切的，头模在鼻子/下巴/后脑方向会超出它。
# 碰撞体 = 躯干常量表 + 头部椭球堆。
#
# 头部椭球必须按**真实蒙皮顶点**拟合，且标记 static：
#   之前用 A_head = global_rest(Head_s)·bind(Head_s) 变换头模顶点，但头模顶点其实绑在**面骨**上，
#   两者 bind 不同 ⇒ 椭球整体偏了 1.6m（中心在 y≈0，而头在 y≈1.66），等于没有头部碰撞。
#   另外头骨原点在 1.6m 外，用它的 pose delta 带动整颗头也是错的 —— 所以 static（不参与 delta）。
func _build_colliders(parts: Dictionary) -> void:
	for d in BODY_COLLIDERS:
		var b := _sk.find_bone(d["bone"])
		if b < 0:
			continue
		var c: Vector3 = d["center"]
		_colliders.append({
			"bone": b, "center": c, "rest_center": c,
			"basis": Basis(), "half": (d["half"] as Vector3) * collision_radius_scale,
		})
	# 头部椭球：按**真实蒙皮顶点在 rest 空间**拟合，与链条同空间。
	# 它只用于「粗筛（判断某关节是否靠近身体）」——真正的推挤由三角形 SDF 完成；
	# 所以必须**准确包住头模**，否则头里的关节不会被判成 near，三角形 SDF 就不会触发。
	# （这曾经是"链条穿过脑袋"的真正原因：旧的 A_head 拟合偏了 1.6m，头模压根没被包住。）
	var head_mi: MeshInstance3D = parts.get("head")
	if head_mi == null or head_mi.skin == null:
		return
	var skin_h: Skin = head_mi.skin
	var nbind: int = mini(_sk.get_bone_count(), skin_h.get_bind_count())
	var mats_h := []
	mats_h.resize(nbind)
	for b2 in nbind:
		mats_h[b2] = _sk.get_bone_global_rest(b2) * skin_h.get_bind_pose(b2)
	var pts := PackedVector3Array()
	for surf in head_mi.mesh.get_surface_count():
		var arr: Array = head_mi.mesh.surface_get_arrays(surf)
		var verts: PackedVector3Array = arr[Mesh.ARRAY_VERTEX]
		var bo: PackedInt32Array = arr[Mesh.ARRAY_BONES]
		var we: PackedFloat32Array = arr[Mesh.ARRAY_WEIGHTS]
		for i in verts.size():
			var acc := Vector3.ZERO
			var ws := 0.0
			for k in 4:
				var w: float = we[i * 4 + k]
				if w <= 0.0:
					continue
				var bi: int = bo[i * 4 + k]
				if bi >= nbind:
					continue
				acc += (mats_h[bi] as Transform3D) * verts[i] * w
				ws += w
			if ws > 0.0:
				pts.append(acc / ws)
	if pts.is_empty():
		return
	var ymin := INF
	var ymax := -INF
	for q in pts:
		ymin = minf(ymin, q.y)
		ymax = maxf(ymax, q.y)
	var yspan: float = maxf(ymax - ymin, 1e-4)
	var slices: int = maxi(HEAD_SLICES, 1)
	for s in slices:
		var y0: float = ymin + yspan * float(s) / float(slices)
		var y1: float = ymin + yspan * float(s + 1) / float(slices)
		var cy: float = (y0 + y1) * 0.5
		var sum := Vector2.ZERO
		var cnt := 0
		for q in pts:
			if q.y >= y0 and q.y <= y1:
				sum += Vector2(q.x, q.z)
				cnt += 1
		if cnt == 0:
			continue
		var cxz: Vector2 = sum / float(cnt)
		var rx := 1e-4
		var rz := 1e-4
		for q in pts:
			if q.y >= y0 and q.y <= y1:
				rx = maxf(rx, absf(q.x - cxz.x))
				rz = maxf(rz, absf(q.z - cxz.y))
		var cen := Vector3(cxz.x, cy, cxz.y)
		var half := Vector3(rx, (y1 - y0) * 0.5, rz) * 1.02
		var k := 1.0
		for q in pts:
			if q.y >= y0 and q.y <= y1:
				k = maxf(k, ((q - cen) / half).length())
		# 用**内切**椭球：乘 head_inset 让它略小于头模。
		# 外扩的椭球会把贴头皮的头发顶飞；内切的则只把"钻进头里"的链推出来，
		# 头部因此重新拥有便宜的碰撞（6 次椭球测试），不必每帧做上千次哈希查询。
		half = half * k * collision_radius_scale * head_inset + Vector3.ONE * head_extra
		_colliders.append({
			"bone": _head, "center": cen, "rest_center": cen,
			"basis": Basis(), "half": half, "static": true,
		})
	var outside := 0
	for i in pts.size():
		var best2 := 1e9
		for ci2 in _colliders.size():
			var cc2: Dictionary = _colliders[ci2]
			if not cc2.get("static", false):
				continue
			best2 = minf(best2, ((pts[i] - (cc2["rest_center"] as Vector3)) / (cc2["half"] as Vector3)).length())
		if best2 > 1.0:
			outside += 1
	# 立刻登记"头部椭球"的下标。**必须在 _build_colliders 里做**：
	# 之前放在 _build_head_probe 里，而那个函数在 head_mesh_collision=false 时会提前返回，
	# 于是 _head_ids 为空 ⇒ _is_head_shape() 永远 false ⇒ 头部椭球照样参与推挤。
	# 头部椭球是"准确包住头模"的胖椭球，拿它推 = 把头发从头皮上顶飞（实测整头发变蓬）。
	_head_ids.clear()
	for i2 in _colliders.size():
		if _colliders[i2].get("static", false):
			_head_ids.append(i2)
	print("[HairSystem] 头部椭球（粗筛用）：%d 段，头模 %d 顶点，未覆盖 %d 个" % [slices, pts.size(), outside])

# =====================================================================
# 严格网格碰撞：体素占据场（由「射线奇偶法」对真实网格实体化）
# ---------------------------------------------------------------------
# 椭球拟合永远只是近似 —— 用户要求「链条不能穿过 mesh」，那就得直接对 mesh 判定。
# 做法：在头发可能触及的三维区域上建体素场，对每根 (x,y) 列沿 +Z 打一条射线，
# 收集与 body+head 三角形的所有交点，排序后把「成对交点之间」的体素标为实心
# （保守：区间两端各外扩一格）。这样「实心」⊇ 真实网格内部，把链条关节推到
# 空格子里，就**保证**它不在 mesh 内部 —— 判据与渲染用的是同一批三角形。
# =====================================================================
const SDF_STEP_FALLBACK := 0.008

func _build_sdf(parts: Dictionary) -> void:
	_sdf_ok = false
	_sdf.clear()
	var meshes := []
	for k in ["body", "head"]:
		var mi: MeshInstance3D = parts.get(k)
		if mi != null and mi.skin != null:
			meshes.append(mi)
	if meshes.is_empty():
		return
	var t0 := Time.get_ticks_usec()
	var step: float = sdf_step
	# 1) 取当前 pose 下的顶点 + 三角形
	var tris := []          # [ax,ay,az,bx,by,bz,cx,cy,cz]
	var mn := Vector3(1e9, 1e9, 1e9)
	var mx := Vector3(-1e9, -1e9, -1e9)
	for mi in meshes:
		var skin: Skin = mi.skin
		var nbind: int = mini(_sk.get_bone_count(), skin.get_bind_count())
		var mats := []
		mats.resize(nbind)
		for b in nbind:
			mats[b] = _sk.get_bone_global_pose(b) * skin.get_bind_pose(b)
		for surf in mi.mesh.get_surface_count():
			var arr: Array = mi.mesh.surface_get_arrays(surf)
			var v: PackedVector3Array = arr[Mesh.ARRAY_VERTEX]
			var idx: PackedInt32Array = arr[Mesh.ARRAY_INDEX]
			var bo: PackedInt32Array = arr[Mesh.ARRAY_BONES]
			var we: PackedFloat32Array = arr[Mesh.ARRAY_WEIGHTS]
			var wp := PackedVector3Array()
			wp.resize(v.size())
			for i in v.size():
				var acc := Vector3.ZERO
				var ws := 0.0
				for k in 4:
					var w: float = we[i * 4 + k]
					if w <= 0.0:
						continue
					var bi: int = bo[i * 4 + k]
					if bi >= nbind:
						continue
					acc += (mats[bi] as Transform3D) * v[i] * w
					ws += w
				wp[i] = acc / ws if ws > 0.0 else v[i]
			var t := 0
			while t + 2 < idx.size():
				var p0: Vector3 = wp[idx[t]]
				var p1: Vector3 = wp[idx[t + 1]]
				var p2: Vector3 = wp[idx[t + 2]]
				t += 3
				var ty := maxf(p0.y, maxf(p1.y, p2.y))
				if ty < sdf_min.y or minf(p0.y, minf(p1.y, p2.y)) > sdf_max.y:
					continue
				if maxf(p0.x, maxf(p1.x, p2.x)) < sdf_min.x or minf(p0.x, minf(p1.x, p2.x)) > sdf_max.x:
					continue
				tris.append([p0, p1, p2])
				mn = mn.min(p0).min(p1).min(p2)
				mx = mx.max(p0).max(p1).max(p2)
	if tris.is_empty():
		return
	# 2) 网格尺寸（xy 用于射线分桶，z 覆盖整个网格保证奇偶法封闭）
	_sdf_min = Vector3(sdf_min.x, sdf_min.y, minf(sdf_min.z, mn.z) - step)
	var zmax: float = maxf(sdf_max.z, mx.z + step)
	_sdf_dim = Vector3i(
		maxi(int(ceil((sdf_max.x - _sdf_min.x) / step)), 1),
		maxi(int(ceil((sdf_max.y - _sdf_min.y) / step)), 1),
		maxi(int(ceil((zmax - _sdf_min.z) / step)), 1))
	_sdf.resize(_sdf_dim.x * _sdf_dim.y * _sdf_dim.z)
	_sdf.fill(0)
	# 3) 三角形按 (x,y) 分桶
	var bstep := 0.03
	var buckets := {}
	for i in tris.size():
		var tr: Array = tris[i]
		var bx0 := int(floor(minf(tr[0].x, minf(tr[1].x, tr[2].x)) / bstep))
		var bx1 := int(floor(maxf(tr[0].x, maxf(tr[1].x, tr[2].x)) / bstep))
		var by0 := int(floor(minf(tr[0].y, minf(tr[1].y, tr[2].y)) / bstep))
		var by1 := int(floor(maxf(tr[0].y, maxf(tr[1].y, tr[2].y)) / bstep))
		for bx in range(bx0, bx1 + 1):
			for by in range(by0, by1 + 1):
				var key := bx * 100003 + by
				if not buckets.has(key):
					buckets[key] = PackedInt32Array()
				var arr2: PackedInt32Array = buckets[key]
				arr2.append(i)
				buckets[key] = arr2
	# 4) 逐列沿 +Z 打射线，成对交点之间填实心（保守外扩一格）
	var zs := []
	var nz: int = _sdf_dim.z
	for ix in _sdf_dim.x:
		for iy in _sdf_dim.y:
			var px: float = _sdf_min.x + (float(ix) + 0.5) * step
			var py: float = _sdf_min.y + (float(iy) + 0.5) * step
			var key2 := int(floor(px / bstep)) * 100003 + int(floor(py / bstep))
			if not buckets.has(key2):
				continue
			zs.clear()
			for ti in (buckets[key2] as PackedInt32Array):
				var tr2: Array = tris[ti]
				var a: Vector3 = tr2[0]
				var b2: Vector3 = tr2[1]
				var c2: Vector3 = tr2[2]
				# 射线 (px,py,·) 沿 +Z 与三角形求交（Möller–Trumbore；d=(0,0,1) 时 d×e2=(-e2.y,e2.x,0)）
				var e1 := b2 - a
				var e2 := c2 - a
				var pv := Vector3(-e2.y, e2.x, 0.0)
				var det := e1.dot(pv)
				if absf(det) < 1e-12:
					continue
				var inv := 1.0 / det
				var s := Vector3(px - a.x, py - a.y, 0.0)
				var uu := s.dot(pv) * inv
				if uu < -1e-6 or uu > 1.0 + 1e-6:
					continue
				var qv := s.cross(e1)
				var vv := qv.z * inv
				if vv < -1e-6 or uu + vv > 1.0 + 1e-6:
					continue
				# 交点的 z（重心插值）+ 朝向符号（用于环绕数）
				var nz2: float = e1.x * e2.y - e1.y * e2.x
				zs.append([a.z + uu * e1.z + vv * e2.z, 1 if nz2 > 0.0 else -1])
			if zs.is_empty():
				continue
			zs.sort_custom(func(x, y): return x[0] < y[0])
			# 奇偶法：只数交点个数，成对之间即为内部。
			# 不用带符号环绕数 —— 这套资产的网格三角形绕向并不一致，按符号累加会互相抵消。
			var base := (ix * _sdf_dim.y + iy) * nz
			var k2 := 0
			while k2 + 1 < zs.size():
				var z0: float = zs[k2][0]
				var z1: float = zs[k2 + 1][0]
				k2 += 2
				if z1 <= z0:
					continue
				var i0 := clampi(int(floor((z0 - _sdf_min.z) / step)), 0, nz - 1)
				var i1 := clampi(int(ceil((z1 - _sdf_min.z) / step)), 0, nz - 1)
				for iz in range(i0, i1 + 1):
					_sdf[base + iz] = 1
	_sdf_ok = true
	# 5) 自检：网格顶点必须全部落在实心体内
	var miss := 0
	var inrange := 0
	for mi in meshes:
		var skin2: Skin = mi.skin
		var nbind2: int = mini(_sk.get_bone_count(), skin2.get_bind_count())
		var mats2 := []
		mats2.resize(nbind2)
		for b in nbind2:
			mats2[b] = _sk.get_bone_global_pose(b) * skin2.get_bind_pose(b)
		for surf in mi.mesh.get_surface_count():
			var arr3: Array = mi.mesh.surface_get_arrays(surf)
			var v3: PackedVector3Array = arr3[Mesh.ARRAY_VERTEX]
			var bo3: PackedInt32Array = arr3[Mesh.ARRAY_BONES]
			var we3: PackedFloat32Array = arr3[Mesh.ARRAY_WEIGHTS]
			var i3 := 0
			while i3 < v3.size():
				var acc3 := Vector3.ZERO
				var ws3 := 0.0
				for k3 in 4:
					var w3: float = we3[i3 * 4 + k3]
					if w3 <= 0.0:
						continue
					var bi3: int = bo3[i3 * 4 + k3]
					if bi3 >= nbind2:
						continue
					acc3 += (mats2[bi3] as Transform3D) * v3[i3] * w3
					ws3 += w3
				if ws3 > 0.0:
					var q3 := acc3 / ws3
					if q3.y >= sdf_min.y and q3.y <= sdf_max.y and absf(q3.x) <= sdf_max.x and q3.z >= _sdf_min.z and q3.z <= (_sdf_min.z + float(_sdf_dim.z) * sdf_step):
						inrange += 1
						if not _sdf_solid(q3):
							miss += 1
				i3 += 7
	print("[HairSystem] 严格碰撞场：%dx%dx%d 体素（步长 %.0fmm，三角形 %d），场内顶点 %d 漏检 %d，耗时 %.0f ms" % [
		_sdf_dim.x, _sdf_dim.y, _sdf_dim.z, step * 1000.0, tris.size(), inrange, miss,
		(Time.get_ticks_usec() - t0) / 1000.0])

func _sdf_solid(p: Vector3) -> bool:
	if not _sdf_ok:
		return false
	var ix := int(floor((p.x - _sdf_min.x) / sdf_step))
	var iy := int(floor((p.y - _sdf_min.y) / sdf_step))
	var iz := int(floor((p.z - _sdf_min.z) / sdf_step))
	if ix < 0 or iy < 0 or iz < 0 or ix >= _sdf_dim.x or iy >= _sdf_dim.y or iz >= _sdf_dim.z:
		return false
	return _sdf[(ix * _sdf_dim.y + iy) * _sdf_dim.z + iz] == 1

# 把点推出实心体：沿 26 个方向逐步外扩，找到第一个空格子
func _sdf_push(pt: Vector3, extra: int = 0) -> Vector3:
	if not _sdf_ok or not _sdf_solid(pt):
		return pt
	var step: float = sdf_step
	for k in range(1, 12):
		var d: float = step * float(k + extra)
		for dz in [-1, 0, 1]:
			for dy in [-1, 0, 1]:
				for dx in [-1, 0, 1]:
					if dx == 0 and dy == 0 and dz == 0:
						continue
					var q := pt + Vector3(float(dx), float(dy), float(dz)) * d
					if not _sdf_solid(q):
						return q
	return pt


# 判定每条链的「头皮固定段」：从根往下，只要关节还在头皮里（含容差）就固定。
# 固定的关节刚性跟随头骨（p = target），完全不参与物理；
# 一旦出去就停止 —— 之后的关节才做 verlet/碰撞/鸟群。
# 判定每条链的「头皮固定段」：从根往下，只要关节还在**头模网格内部**（含容差）就固定。
# 这些关节刚性跟随头骨（p = target），完全不参与 verlet/碰撞/鸟群；
# 第一个走出头模的关节之后的全部关节才参与物理。
#
# 为什么必须用真实头模而不是椭球：头部椭球把脸/下巴/脖子都包进去了，
# 用椭球判定会把"垂在脸前"的头发也当成头皮里，于是整条前发全被钉死。
# 头模探针（rest 空间）：三角形 + 4cm 空间哈希。
# 头部碰撞与"头皮固定段"判定都基于**真实头模**而不是椭球 ——
# 头部椭球比头模胖，直接拿椭球推会把贴着头皮的头发顶飞。
var _head_probe_ok := false

func _build_head_probe() -> void:
	_head_tris.clear()
	_head_grid.clear()
	_head_probe_ok = false
	# 这个探针只在**构建期**用于「头皮固定段」分类（_compute_attachment），
	# 不进逐帧热路径 —— 无论 head_mesh_collision 开关都要建。
	# 之前跟着那个开关提前返回 ⇒ _attach 永远只有 1 个关节 ⇒
	# 发卡嵌在头皮里的第 1~3 个关节没被钉住，转头时正是它们带着链条扫进颅骨。

	var head_mi: MeshInstance3D = _parts.get("head")
	if head_mi == null or head_mi.skin == null:
		return
	var skin: Skin = head_mi.skin
	var nbind: int = mini(_sk.get_bone_count(), skin.get_bind_count())
	var mats := []
	mats.resize(nbind)
	for b in nbind:
		mats[b] = _sk.get_bone_global_rest(b) * skin.get_bind_pose(b)
	var cell: float = _head_cell
	for surf in head_mi.mesh.get_surface_count():
		var arr: Array = head_mi.mesh.surface_get_arrays(surf)
		var v: PackedVector3Array = arr[Mesh.ARRAY_VERTEX]
		var idx: PackedInt32Array = arr[Mesh.ARRAY_INDEX]
		var bo: PackedInt32Array = arr[Mesh.ARRAY_BONES]
		var we: PackedFloat32Array = arr[Mesh.ARRAY_WEIGHTS]
		var wp := PackedVector3Array()
		wp.resize(v.size())
		for i in v.size():
			var acc := Vector3.ZERO
			var ws := 0.0
			for k in 4:
				var w: float = we[i * 4 + k]
				if w <= 0.0:
					continue
				var bi: int = bo[i * 4 + k]
				if bi >= nbind:
					continue
				acc += (mats[bi] as Transform3D) * v[i] * w
				ws += w
			wp[i] = acc / ws if ws > 0.0 else v[i]
		var t := 0
		while t + 2 < idx.size():
			var p0: Vector3 = wp[idx[t]]
			var p1: Vector3 = wp[idx[t + 1]]
			var p2: Vector3 = wp[idx[t + 2]]
			t += 3
			var ti := _head_tris.size()
			_head_tris.append([p0, p1, p2])
			var mn: Vector3 = p0.min(p1).min(p2)
			var mx: Vector3 = p0.max(p1).max(p2)
			for gx in range(int(floor(mn.x / cell)), int(floor(mx.x / cell)) + 1):
				for gy in range(int(floor(mn.y / cell)), int(floor(mx.y / cell)) + 1):
					for gz in range(int(floor(mn.z / cell)), int(floor(mx.z / cell)) + 1):
						var key := gx * 1000003 + gy * 10007 + gz
						if not _head_grid.has(key):
							_head_grid[key] = PackedInt32Array()
						var bucket: PackedInt32Array = _head_grid[key]
						bucket.append(ti)
						_head_grid[key] = bucket
	_head_probe_ok = _head_tris.size() > 0
	_head_ids.clear()
	for i in _colliders.size():
		if _colliders[i].get("static", false):
			_head_ids.append(i)
	print("[HairSystem] 头模探针：%d 个三角形（rest 空间），头部椭球 %d 颗" % [_head_tris.size(), _head_ids.size()])

# 判定每条链的「头皮固定段」：从根往下，只要关节还在**头模网格内部**就固定。
# 这些关节刚性跟随头骨、完全不参与物理；第一个走出去之后的关节才参与物理。
func _compute_attachment() -> void:
	_attach.clear()
	for ci in _chains.size():
		var q: PackedVector3Array = _chains[ci]["q"]
		var k := 1
		for j in q.size():
			if _head_probe_ok and _point_in_head(q[j], _head_tris, _head_grid, _head_cell):
				k = j + 1
			else:
				break
		# 发根**必须**刚性绑定在头皮上（用户要求"根部和身体绑定"）：
		# 现实中发根牢牢长在头皮里，不是单点铰接。
		# 只钉 1 个关节时，第 2 个关节在快速转头中会被 FTL 甩进颅骨 —— 扫掠穿透的主因。
		_attach.append(clampi(maxi(k, scalp_min_attach), 1, q.size()))
	var fixed := 0
	for k2 in _attach:
		fixed += k2
	print("[HairSystem] 头皮固定段：%d 条链固定 %d 个关节 / 共 %d（每条 %s）" % [
		_attach.size(), fixed, _chains.size() * _chains[0]["q"].size(), str(_attach)])
# 点是否在头模内部（射线奇偶；用 3x3x3 邻域三角形加速）
func _point_in_head(p: Vector3, tris: Array, grid: Dictionary, cell: float) -> bool:
	var d := Vector3(1.0, 0.371, 0.193).normalized()
	var gx := int(floor(p.x / cell))
	var gy := int(floor(p.y / cell))
	var gz := int(floor(p.z / cell))
	var cnt := 0
	for dx in range(-1, 2):
		for dy in range(-1, 2):
			for dz in range(-1, 2):
				var key := (gx + dx) * 1000003 + (gy + dy) * 10007 + (gz + dz)
				if not grid.has(key):
					continue
				for ti in (grid[key] as PackedInt32Array):
					var tr: Array = tris[ti]
					var a: Vector3 = tr[0]
					var e1: Vector3 = (tr[1] as Vector3) - a
					var e2: Vector3 = (tr[2] as Vector3) - a
					var h := d.cross(e2)
					var det := e1.dot(h)
					if absf(det) < 1e-12:
						continue
					var inv := 1.0 / det
					var s: Vector3 = p - a
					var u := s.dot(h) * inv
					if u < 0.0 or u > 1.0:
						continue
					var qv := s.cross(e1)
					var vv := d.dot(qv) * inv
					if vv < 0.0 or u + vv > 1.0:
						continue
					if e2.dot(qv) * inv > 1e-6:
						cnt += 1
	return (cnt % 2) == 1
# 静止发链预处理：所有关节（含被钉死的根）先推出碰撞体一次。
# 这样"钉死的根"不会落在头里，权重也是按修正后的链形算的，静置即可穿模为 0。
func _relax_rest_chains() -> void:
	_build_shapes(collision_margin)
	var moved := 0
	var worst := 0.0
	for ci in _chains.size():
		# 半径表还没算（它依赖权重、权重又依赖链形）；这里先用统一的薄壳厚度做预推
		var q: PackedVector3Array = _chains[ci]["q"]
		var na: int = _attach[ci] if ci < _attach.size() else 1
		# ① 刚性发根段（j < na）也要做体外投影！
		#    它们是「随头骨刚性运动」的构造性位置 —— 构建期若嵌在头皮里，
		#    之后永远嵌在里面，永远不可能靠运行时碰撞修正（因为不参与物理）。
		#    这就是"头顶的链条穿到脑袋里"的根因：发根段从链形生成起就在颅骨内。
		for j in range(na):
			q[j] = _project_out(q[j], contact_margin)
		# ② 自由段：推出碰撞体
		for j in range(na, q.size()):
			var before := q[j]
			for t in _joint_neighbours:
				q[j] = _push_one(q[j], _joint_near_for(ci, j, t), 0.008)
			# 注意：静止预推**不做**顶点推挤 —— 发片本来就贴着身体，
			# 预推时推它等于把整头发从身上掀开（实测静止漂移 130mm）。
			# 顶点碰撞只在运行时约束"运动后插进去"的节点。
			# 预推绝不允许把关节推进头模里（碰撞体是近似，推挤方向可能指向头内）
			if _head_probe_ok and _point_in_head(q[j], _head_tris, _head_grid, _head_cell):
				q[j] = before
			# 限制单次位移：发片内侧本来就"长在头里"的关节不该被整段扯出去
			# （不限制时实测有 178mm 的位移，整头发型会被扯变形）
			var dv := q[j] - before
			if dv.length() > max_rest_push:
				q[j] = before + dv.normalized() * max_rest_push
			if q[j].distance_squared_to(before) > 1e-10:
				moved += 1
				worst = maxf(worst, q[j].distance_to(before))
		_chains[ci]["q"] = q
		_chains[ci]["p"] = q.duplicate()
		_chains[ci]["prev"] = q.duplicate()
		_chains[ci]["target"] = q.duplicate()
		# 关节被移动后**必须重算段长**：段长是 _make_chain 时按原折线算的，
		# 不重算的话「不伸缩约束」就会和链形对不上 —— 表现为剧烈拉伸（实测 83%）。
		var lens := PackedFloat32Array()
		var total := 0.0
		for i in q.size() - 1:
			var l := q[i].distance_to(q[i + 1])
			lens.append(l)
			total += l
		_chains[ci]["len"] = lens
		_chains[ci]["total"] = total
	if moved > 0:
		print("[HairSystem] 静止发链预推：%d 个关节被推出碰撞体（最大 %.1fmm），段长已重算" % [moved, worst * 1000.0])

# 取第 ci 条链第 j 个关节的第 t 个邻近碰撞体（_joint_near 还没建时按静止位姿现算）
func _joint_near_for(ci: int, j: int, t: int) -> int:
	if ci < _joint_near.size():
		var nn: PackedInt32Array = _joint_near[ci]
		return nn[j * _joint_neighbours + t]
	var q: Vector3 = _chains[ci]["q"][j]
	var nn2 := _nearest_shapes(q, _joint_neighbours)
	return nn2[t]

# 每个链关节的「发片厚度」：该关节支配的网格顶点，到关节静止位置的距离（85 分位）。
# 碰撞时把身体椭球按这个半径外扩，链心被推开这么多，可见的发片表面才刚好贴住身体。
# 每个关节的「发片厚度」= 该关节支配的顶点，比关节本身更深地插进身体多少。
# 注意不能用「顶点到关节的距离」：那是发片的横向尺寸（可能是几十厘米），
# 拿它当碰撞半径会把整条头发从身体上顶开，看起来就是"头发浮在空中"。
# 只对身体椭球（非 static）求有符号深度：头部椭球是"精确包围盒"，不参与厚度估算
func _signed_depth_body(p: Vector3) -> float:
	var best := -1e9
	for i in _colliders.size():
		if _colliders[i].get("static", false):
			continue
		best = maxf(best, _shape_signed(p, i))
	return best

func _compute_radii() -> void:
	_build_shapes(0.0)
	var buckets := []
	for ci in _chains.size():
		var arr := []
		for j in _chains[ci]["q"].size():
			arr.append([])
		buckets.append(arr)
	for mi in _hair_meshes:
		var skin: Skin = mi.skin
		if skin == null:
			continue
		for surf in mi.mesh.get_surface_count():
			var a: Array = mi.mesh.surface_get_arrays(surf)
			var v: PackedVector3Array = a[Mesh.ARRAY_VERTEX]
			var bo: PackedInt32Array = a[Mesh.ARRAY_BONES]
			var we: PackedFloat32Array = a[Mesh.ARRAY_WEIGHTS]
			for i in v.size():
				var best := -1
				var bw := 0.0
				for k in 4:
					var w: float = we[i * 4 + k]
					if w > bw:
						bw = w
						best = bo[i * 4 + k]
				if not _bone_rc.has(best):
					continue
				var rc: Vector2i = _bone_rc[best]
				var q: Vector3 = _A * v[i]
				var jp: Vector3 = _chains[rc.x]["q"][rc.y]
				# 正值 = 该顶点比关节更深入身体；关节只需往外让出这么多，发片面就刚好贴住身体。
				# **只对身体椭球算**：头部椭球(static)现在准确包住了头模，
				# 把"发片在头皮里有多深"也算进来的话，rad 会暴涨到 25mm 上限，
				# 于是链心被顶离身体 → 整头发变蓬松（实测）。
				buckets[rc.x][rc.y].append(maxf(_signed_depth_body(q) - _signed_depth_body(jp), 0.0))
	_radius.clear()
	for ci in _chains.size():
		var out := PackedFloat32Array()
		for j in buckets[ci].size():
			var ds: Array = buckets[ci][j]
			if ds.is_empty():
				out.append(0.0)
				continue
			ds.sort()
			var idx: int = clampi(int(float(ds.size()) * radius_percentile), 0, ds.size() - 1)
			out.append(clampf(float(ds[idx]), 0.0, max_hair_radius))
		_radius.append(out)

# 是否真的在头模内部：射线奇偶判定 **且** 落在包含头模的椭球内。
# 单用射线判定会误判 —— 头模底部（脖子切口）是开放的，切口附近的点奇偶性会翻错，
# 于是「贴在头皮上」的头发被误判成"在头里"并被推走。
func _head_inside(pt: Vector3) -> bool:
	if not _head_probe_ok:
		return false
	if not _point_in_head(pt, _head_tris, _head_grid, _head_cell):
		return false
	for hi in _head_ids:
		if _shape_signed(pt, hi) > 0.0:
			return true
	return false

func _is_head_shape(i: int) -> bool:
	for hi in _head_ids:
		if hi == i:
			return true
	return false

func _posed_collider(c: Dictionary) -> Array:
	# 头部椭球（static）不参与 pose delta：头骨原点在 1.6m 外，用它带动整颗头本来就是错的；
	# 头部碰撞实际由 _point_in_head 的真实头模判定完成，椭球只用于邻近筛选。
	if c.get("static", false):
		return [c["rest_center"], Basis()]
	var b: int = c["bone"]
	var delta: Transform3D = _sk.get_bone_global_pose(b) * _sk.get_bone_global_rest(b).affine_inverse()
	# 保留 delta.basis 的缩放（不能 orthonormalize）：捏人滑杆会缩放骨骼（头大小/胸/腰…），
	# 丢掉缩放的话身体变大而碰撞体不变，头发就会插进去。
	return [delta * (c["rest_center"] as Vector3), delta.basis]
func _reset() -> void:
	for c in _chains:
		var q: PackedVector3Array = c["q"]
		c["p"] = q.duplicate()
		c["prev"] = q.duplicate()
	_first_frame = true
	metric_max_stretch = 0.0
	metric_min_sep = 999.0
	metric_max_pen = 0.0

func set_display_mode(m: int) -> void:
	display_mode = clampi(m, 0, 2)
	apply_display()

func set_enabled(on: bool) -> void:
	enabled = on
	if _sk == null:
		return
	if on:
		reset_physics()
	else:
		_clear_overrides()
		for c in _chains:
			c["p"] = (c["q"] as PackedVector3Array).duplicate()
			c["prev"] = (c["p"] as PackedVector3Array).duplicate()

func reset_physics() -> void:
	_clear_overrides()
	_reset()

func _clear_overrides() -> void:
	# Godot 4.7 只有「清空全部」的 API（本系统是骨架 override 的唯一使用者，安全）
	_sk.clear_bones_global_pose_override()

func _exit_tree() -> void:
	if _sk != null and not _bones.is_empty():
		_clear_overrides()


# =====================================================================
# XPBD（Extended Position Based Dynamics）头发求解器
# ---------------------------------------------------------------------
# 与旧 PBD 的本质区别（对应三个顽疾）：
#   ① 子步进：8 个子步 × 每步 1~2 次迭代。单步位移小 8 倍 ⇒
#      快速转头时链不再一步跨过颅骨（解决扫掠穿透）。
#   ② 速度只在子步末重算一次：v = (p - p_子步起点) / h。
#      约束效果自动成为速度，没有任何"位移注入速度"的通道 ⇒
#      无风时不抖（解决持续乱晃）。
#   ③ 发根段刚性绑定（质量倒数 w=0），随头骨刚性运动 ⇒ 根部与身体绑定。
# 弯曲约束保持局部曲率（形状记忆的正确形式），重力主导下垂贴身。
# =====================================================================
@export var use_xpbd := true
@export var xpbd_substeps := 4
@export var xpbd_iters := 2
@export_range(0.0, 1.0) var xpbd_bend := 0.25      # 弯曲约束每次迭代的修正比例
@export_range(0.0, 0.5) var xpbd_damping := 0.04   # 每子步速度衰减

var _xp_v: Array = []                              # 每链速度
var _xp_pp: Array = []                             # 每链：子步起点位置
var _xp_ok := false

# ===========================================================================
# XPBD 求解器（Macklin 2016）：
#   · 头皮附着（软 α_att）：发根柔性绑在头皮锚点，转头不再把链甩进颅骨
#   · 距离（硬 α=0）：不可伸缩
#   · 弯曲（软 α_bend）：形状记忆
#   · 碰撞（硬单边）：惰性 SDF，sd ≥ margin
#   · λ 跨迭代累加、子步末清零；速度 = 实际位移/h（无能量注入通道）
# 由此从机制上消除：无风乱抖、链长被拉成橡皮筋、快速转头扫掠穿透。
# ===========================================================================
func _xpbd_ensure() -> void:
	var need := _xp_v.size() != _chains.size() or _chains.is_empty()
	if not need:
		for ci in _chains.size():
			if (_xp_v[ci] as PackedVector3Array).size() != _chains[ci]["q"].size():
				need = true
				break
			if (_lam_l[ci] as PackedFloat32Array).size() != _chains[ci]["len"].size():
				need = true
				break
	if not need:
		return
	_xp_v.clear()
	_xp_pp.clear()
	_lam_l.clear()
	_lam_b.clear()
	_lam_c.clear()
	for ci in _chains.size():
		var n: int = _chains[ci]["q"].size()
		var vv := PackedVector3Array(); vv.resize(n)
		var pp := PackedVector3Array(); pp.resize(n)
		_xp_v.append(vv)
		_xp_pp.append(pp)
		var ll := PackedFloat32Array(); ll.resize(maxi(n - 1, 0))
		var lb := PackedFloat32Array(); lb.resize(n)
		var lc := PackedFloat32Array(); lc.resize(n)
		_lam_l.append(ll)
		_lam_b.append(lb)
		_lam_c.append(lc)
	_xp_ok = _chains.size() > 0

# ---------------------------------------------------------------------------
# XPBD 主循环（Macklin 2016）：所有约束带拉格朗日乘子 λ（跨迭代累加、子步末清零），
# 刚度由 compliance 控制 ⇒ 约束之间按柔顺度分配修正量，不会互相注入能量。
# 这是普通 PBD 抖动的机制性解法。
# ---------------------------------------------------------------------------
func _xpbd_frame(dt: float) -> void:
	_xpbd_ensure()
	if not _xp_ok:
		return
	var n_sub: int = maxi(xpbd_substeps, 1)
	var h := dt / float(n_sub)
	var h2 := h * h
	var g := _g_local * gravity + wind
	for s in n_sub:
		# ① 记录子步起点 + 预测（半隐式欧拉）
		for ci in _chains.size():
			var ch: Dictionary = _chains[ci]
			var p: PackedVector3Array = ch["p"]
			var pp: PackedVector3Array = _xp_pp[ci]
			var v: PackedVector3Array = _xp_v[ci]
			var tg: PackedVector3Array = ch["target"]
			for j in p.size():
				pp[j] = p[j]
			for j in range(1, p.size()):
				v[j] += g * h
				p[j] += v[j] * h
			p[0] = tg[0]
			v[0] = Vector3.ZERO
		# ② λ 清零（每个子步重新累积）
		for ci in _chains.size():
			var ll: PackedFloat32Array = _lam_l[ci]
			for j in ll.size(): ll[j] = 0.0
			var lb: PackedFloat32Array = _lam_b[ci]
			for j in lb.size(): lb[j] = 0.0
			var lc: PackedFloat32Array = _lam_c[ci]
			for j in lc.size(): lc[j] = 0.0
		# ③ 约束迭代（XPBD）
		for it in xpbd_iters:
			_xpbd_attach(h2)
			_xpbd_distance(h2)
			_xpbd_bend(h2)
			if collision_enabled:
				_xpbd_collision(h2)
		# ④ 速度重算 + 阻尼（XPBD 标准收尾）
		for ci in _chains.size():
			var ch: Dictionary = _chains[ci]
			var p: PackedVector3Array = ch["p"]
			var pp: PackedVector3Array = _xp_pp[ci]
			var v: PackedVector3Array = _xp_v[ci]
			var na: int = _attach[ci] if ci < _attach.size() else 1
			var dampf := 1.0 - clampf(xpbd_damping, 0.0, 0.9)
			for j in range(1, p.size()):
				v[j] = (p[j] - pp[j]) / h
				v[j] *= dampf
			ch["p"] = p

# 头皮附着约束（软）：发根粒子柔性绑在头皮锚点上。
# 完全刚性会在转头时把整条链甩进颅骨；毛囊有微小弹性，柔性更真实。
func _xpbd_attach(h2: float) -> void:
	var at := attach_compliance / h2 if attach_compliance > 0.0 else 0.0
	for ci in _guides:
		var ch: Dictionary = _chains[ci]
		var p: PackedVector3Array = ch["p"]
		var tg: PackedVector3Array = ch["target"]
		var d := p[0] - tg[0]
		var C := d.length()
		if C < 1e-9:
			continue
		var denom := 1.0 + at
		var dl := -C / denom
		p[0] = p[0] + d.normalized() * dl
# 距离约束（不可伸缩，α=0 刚性）
func _xpbd_distance(h2: float) -> void:
	for ci in _guides:
		var ch: Dictionary = _chains[ci]
		var p: PackedVector3Array = ch["p"]
		var lens: PackedFloat32Array = ch["len"]
		var lam: PackedFloat32Array = _lam_l[ci]
		var na: int = _attach[ci] if ci < _attach.size() else 1
		for j in range(1, p.size()):
			var a := p[j - 1]
			var b := p[j]
			var d := b - a
			var l := d.length()
			if l < 1e-9:
				continue
			var n := d / l
			var C := l - lens[j - 1]
			var wa := 0.0 if (j - 1) < na else 1.0
			var wb := 0.0 if j < na else 1.0
			var ws := wa + wb
			if ws <= 0.0:
				continue
			var idx := j - 1
			var dlam := -C / ws
			lam[idx] += dlam
			p[j - 1] = a - n * (dlam * wa)
			p[j] = b + n * (dlam * wb)

# 弯曲约束（软）：把 (j-1, j+1) 的距离拉回静止值，只动两端
func _xpbd_bend(h2: float) -> void:
	if bend_stiffness <= 0.0:
		return
	for ci in _guides:
		var ch: Dictionary = _chains[ci]
		var p: PackedVector3Array = ch["p"]
		var bend: PackedFloat32Array = ch["bend"]
		var na: int = _attach[ci] if ci < _attach.size() else 1
		for j in range(1, p.size() - 1):
			if bend[j] <= 0.0:
				continue
			var a := p[j - 1]
			var cc := p[j + 1]
			var d := cc - a
			var dl := d.length()
			if dl < 1e-9:
				continue
			var corr := d * ((dl - bend[j]) / dl) * 0.5 * bend_stiffness
			if j - 1 >= na:
				p[j - 1] = a + corr
			if j + 1 >= na:
				p[j + 1] = cc - corr

# 碰撞约束（硬，单边）：惰性 SDF 查询；sd < margin 就沿外法线推到 margin。
# 深陷（>15mm 或到顶点 >25mm）时法线不可靠，改用"远离部位质心"方向。
func _xpbd_collision(h2: float) -> void:
	if not _vtx_ok:
		return
	for ci in _guides:
		var ch: Dictionary = _chains[ci]
		var p: PackedVector3Array = ch["p"]
		var na: int = _attach[ci] if ci < _attach.size() else 1
		for j in range(na, p.size()):
			var r: Array = _sdf_at(p[j])
			var sd: float = r[0]
			if sd >= contact_margin:
				continue
			var n: Vector3 = r[1]
			if r[2] > 0.025 or sd < -0.012:
				var ref := _head_center if p[j].y > _head_center.y else _body_center
				var esc := p[j] - ref
				if esc.length_squared() > 1e-8:
					n = esc.normalized()
			var delta := n * (contact_margin - sd)
			var dl2 := delta.length()
			if dl2 > _max_push:
				delta = delta * (_max_push / dl2)
			p[j] = p[j] + delta
func _derive_interpolated() -> void:
	# 插值链由两条导引链混合而来；混合结果可能落在身体里（两链分居身体两侧时），
	# 所以混合后也要按各自厚度推出去。
	if collision_enabled:
		_build_shapes(collision_margin)
	for ci in _chains.size():
		var c: Dictionary = _chains[ci]
		if c["guide"]:
			continue
		var qa: PackedVector3Array = _chains[c["ga"]]["p"]
		var qb: PackedVector3Array = _chains[c["gb"]]["p"]
		var f: float = c["blend"]
		var rad: PackedFloat32Array = _radius[ci]
		var nn: PackedInt32Array = _joint_near[ci]
		var out := PackedVector3Array()
		var n := mini(qa.size(), qb.size())
		var qv: PackedVector3Array = c["q"]
		var na: int = _attach[ci] if ci < _attach.size() else 1
		for j in n:
			var pt: Vector3
			if j < na:
				pt = _head_pose * qv[j]      # 插值链的固定段同样钉在头皮上
			else:
				pt = qa[j].lerp(qb[j], f)
				# 插值链 = 两条导引链当前位置的 lerp，两链分居身体两侧时连线必然穿过身体，
				# 所以它们**必须**有自己的碰撞约束。SDF 是 O(1)，28 条链也付得起。
				if collision_enabled:
					pt = _project_out(pt, contact_margin)
			out.append(pt)
		c["p"] = out
		c["prev"] = out.duplicate()


# =====================================================================
# 网格级修补：链心推出身体还不够 —— 发片是「面」，离链心有几厘米。
# 这里直接对「静止时在体外、但离身体很近」的顶点做真实蒙皮计算，
# 谁的蒙皮位置插进身体，就把驱动它的链关节往外交同样的位移，再重投影链长。
# =====================================================================
func _collect_candidates() -> void:
	_cand_m.clear(); _cand_i.clear(); _cand_b.clear(); _cand_w.clear(); _cand_v.clear(); _cand_n.clear()
	_build_shapes(0.0)
	for mii in _hair_meshes.size():
		var mi: MeshInstance3D = _hair_meshes[mii]
		var skin: Skin = mi.skin
		if skin == null:
			continue
		var flags: PackedByteArray = _rest_outside[mii] if mii < _rest_outside.size() else PackedByteArray()
		var mats := _bone_matrices(mi)
		var base := 0
		for surf in mi.mesh.get_surface_count():
			var a: Array = mi.mesh.surface_get_arrays(surf)
			var v: PackedVector3Array = a[Mesh.ARRAY_VERTEX]
			var bo: PackedInt32Array = a[Mesh.ARRAY_BONES]
			var we: PackedFloat32Array = a[Mesh.ARRAY_WEIGHTS]
			for i in v.size():
				if flags.size() <= base + i or flags[base + i] != 1:
					continue
				var acc := Vector3.ZERO
				var ws := 0.0
				for k in 4:
					var w: float = we[i * 4 + k]
					if w <= 0.0:
						continue
					acc += (mats[bo[i * 4 + k]] as Transform3D) * v[i] * w
					ws += w
				if ws <= 0.0:
					continue
				var p := acc / ws
				if _signed_depth(p) < -candidate_range:
					continue   # 离身体还远，不会插进去
				var usable := true
				for k in 4:
					if we[i * 4 + k] > 0.0 and not _bone_rc.has(bo[i * 4 + k]):
						usable = false
				if not usable:
					continue
				_cand_m.append(mii)
				_cand_i.append(i)
				for k in 4:
					_cand_b.append(bo[i * 4 + k])
					_cand_w.append(we[i * 4 + k])
				_cand_v.append(v[i])
				var nn := _nearest_shapes(p, _cand_neighbours)
				for t in _cand_neighbours:
					_cand_n.append(nn[t])
			base += v.size()

# 便宜的「最近几颗椭球」粗排：比较点到球心的距离与椭球外接球半径
func _nearest_shapes(pt: Vector3, k: int) -> PackedInt32Array:
	var best_d := []
	var best_i := []
	for i in _sh_c.size():
		var d: float = (pt - _sh_c[i]).length_squared() - _sh_r[i] * _sh_r[i]
		var pos := best_d.size()
		for t in best_d.size():
			if d < best_d[t]:
				pos = t
				break
		if pos < k:
			best_d.insert(pos, d)
			best_i.insert(pos, i)
			if best_d.size() > k:
				best_d.pop_back()
				best_i.pop_back()
	var out := PackedInt32Array()
	for t in k:
		out.append(int(best_i[t]) if t < best_i.size() else 0)
	return out

# 单颗椭球的有符号距离（>0 在内，<0 在外）
func _shape_signed(pt: Vector3, i: int) -> float:
	var half: Vector3 = _sh_h[i]
	var u: Vector3 = (_sh_bi[i] as Basis) * (pt - _sh_c[i])
	var s := Vector3(u.x / half.x, u.y / half.y, u.z / half.z)
	var sl := s.length()
	if sl < 1e-6:
		return 0.0
	return u.length() * (1.0 / sl - 1.0)

# 有符号「到身体表面」的距离：>0 在体内，<0 在体外
# 每个链关节预先挑出最近的几颗椭球：碰撞热路径只测这几颗，不再 19 颗全测
func _compute_joint_neighbours() -> void:
	_joint_near.clear()
	_build_shapes(0.0)
	for ci in _chains.size():
		var q: PackedVector3Array = _chains[ci]["q"]
		var out := PackedInt32Array()
		for j in q.size():
			var near := []
			for si in _sh_c.size():
				# 头部椭球**必须留在列表里**：它现在用内切版本参与推挤。
				# 之前把它排除后，推挤循环看不到头部 ⇒ 头部等于没有碰撞 ⇒
				# 头顶的链直接钻进 mesh（实测非发根关节 271 个在体内）。
				near.append([_shape_signed(q[j], si), si])
			near.sort_custom(func(x, y): return x[0] > y[0])
			for t in _joint_neighbours:
				out.append(int(near[t][1]) if t < near.size() else 0)
		_joint_near.append(out)

func _signed_depth(pt: Vector3) -> float:
	var best := -1e9
	for i in _sh_c.size():
		var half: Vector3 = _sh_h[i]
		var u: Vector3 = (_sh_bi[i] as Basis) * (pt - _sh_c[i])
		var s := Vector3(u.x / half.x, u.y / half.y, u.z / half.z)
		var sl := s.length()
		if sl < 1e-6:
			return 0.0
		# 沿径向到椭球面的距离（负=在外）
		best = maxf(best, u.length() * (1.0 / sl - 1.0))
	return best

func _mesh_repair() -> void:
	if not mesh_repair or _cand_m.is_empty():
		return
	var nj: int = joints_per_chain
	var corr := []
	var ccnt := PackedInt32Array()
	corr.resize(_chains.size() * nj)
	ccnt.resize(_chains.size() * nj)
	for it in mesh_repair_iterations:
		_build_shapes(0.0)
		_cand_refresh = (_cand_refresh + 1) % maxi(candidate_refresh_interval, 1)
		for i in corr.size():
			corr[i] = Vector3.ZERO
			ccnt[i] = 0
		var any := false
		var mats_cache := []
		for mii in _hair_meshes.size():
			mats_cache.append(_bone_matrices(_hair_meshes[mii]))
		for ci in _cand_m.size():
			var mii: int = _cand_m[ci]
			var mats: Array = mats_cache[mii]
			var vv: Vector3 = _cand_v[ci]
			var pos := Vector3.ZERO
			var ws := 0.0
			for k in 4:
				var wk: float = _cand_w[ci * 4 + k]
				if wk <= 0.0:
					continue
				pos += (mats[_cand_b[ci * 4 + k]] as Transform3D) * vv * wk
				ws += wk
			if ws <= 0.0:
				continue
			pos /= ws
			if _cand_refresh == 0:
				# 定期按当前位置刷新「最近的几颗椭球」（粗排用便宜的中心距，不开方）
				var nn := _nearest_shapes(pos, _cand_neighbours)
				for t in _cand_neighbours:
					_cand_n[ci * _cand_neighbours + t] = nn[t]
			var q := pos
			for t in _cand_neighbours:
				q = _push_one(q, _cand_n[ci * _cand_neighbours + t], 0.0)
			var dl := q - pos
			if dl.length_squared() < 1e-10:
				continue
			any = true
			# 把顶点位移换算成「各驱动关节需要移动多少」。
			# 顶点位置 ≈ Σ w_k · 关节位置，要让顶点移动 dl，最小范数解是 Δ_k = w_k · dl / Σw_k²。
			# 少了 1/Σw_k² 这一步就会**系统性欠修正**：一个 0.5/0.5 权重的顶点只被推了一半，
			# 权重越分散（插值链越多）差得越多 —— 转头时插值链一带的顶点就是这么穿进头里的。
			var sw2 := 0.0
			for k in 4:
				var wk2: float = _cand_w[ci * 4 + k]
				if wk2 <= 0.0001:
					continue
				if not _bone_rc.has(_cand_b[ci * 4 + k]):
					continue
				var rc0: Vector2i = _bone_rc[_cand_b[ci * 4 + k]]
				if rc0.y < (_attach[rc0.x] if rc0.x < _attach.size() else 1):
					continue   # 头皮固定段的关节推不动，不参与
				sw2 += wk2 * wk2
			if sw2 <= 1e-6:
				continue
			var dk := dl / sw2
			# 按蒙皮权重分摊给驱动它的链关节（虚拟链再按混合权重分摊回两条导引链）
			for k in 4:
				var w: float = _cand_w[ci * 4 + k]
				if w <= 0.0001:
					continue
				var b: int = _cand_b[ci * 4 + k]
				if not _bone_rc.has(b):
					continue
				var rc: Vector2i = _bone_rc[b]
				var ch: Dictionary = _chains[rc.x]
				if ch["guide"]:
					if rc.y < (_attach[rc.x] if rc.x < _attach.size() else 1):
						continue
					corr[rc.x * nj + rc.y] = corr[rc.x * nj + rc.y] + dk * w
					ccnt[rc.x * nj + rc.y] = ccnt[rc.x * nj + rc.y] + 1
				else:
					var f: float = ch["blend"]
					var ca: int = ch["ga"]
					var cb: int = ch["gb"]
					if rc.y > 0:
						corr[ca * nj + rc.y] = corr[ca * nj + rc.y] + dk * (w * f)
						corr[cb * nj + rc.y] = corr[cb * nj + rc.y] + dk * (w * (1.0 - f))
						ccnt[ca * nj + rc.y] = ccnt[ca * nj + rc.y] + 1
						ccnt[cb * nj + rc.y] = ccnt[cb * nj + rc.y] + 1
		if not any:
			break
		for ci in _chains.size():
			var ch: Dictionary = _chains[ci]
			if not ch["guide"]:
				continue
			var p: PackedVector3Array = ch["p"]
			var moved := false
			for j in range(1, p.size()):
				var n2: int = ccnt[ci * nj + j]
				if n2 <= 0:
					continue
				# 取平均而不是求和：同一个关节常被许多顶点同时驱动，直接累加会过冲振荡。
				# 平均后每帧位移有界，再乘松弛系数。
				var c: Vector3 = corr[ci * nj + j] / float(n2) * mesh_repair_relaxation
				if c.length_squared() > 1e-12:
					p[j] = p[j] + c
					moved = true
			if moved:
				ch["p"] = p
		# 链长是不伸缩约束，推完必须重投影
		_solve_lengths_exact()
	_derive_interpolated()


# =====================================================================
# 发链可视化：关掉头发网格，直接把物理链条画成发丝（用于检查物理质量）
# =====================================================================
func _build_chain_view() -> void:
	if _chain_mi != null:
		return
	_chain_mi = MeshInstance3D.new()
	_chain_mi.name = "HairChains"
	_chain_im = ImmediateMesh.new()
	_chain_mi.mesh = _chain_im
	var m := StandardMaterial3D.new()
	m.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	m.vertex_color_use_as_albedo = true
	m.cull_mode = BaseMaterial3D.CULL_DISABLED
	m.no_depth_test = false
	_chain_mi.material_override = m
	_chain_mi.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	_sk.add_child(_chain_mi)
	apply_display()

# 切换显示模式（UI 调用）
func apply_display() -> void:
	var show_mesh := display_mode != 1
	for mi in _hair_meshes:
		mi.visible = show_mesh
		if show_mesh:
			mi.visible = true
	if _chain_mi != null:
		_chain_mi.visible = display_mode != 0
		if _chain_mi.visible:
			_update_chain_mesh()

func _chain_tint(ci: int, guide: bool) -> Color:
	if guide:
		# 导引链：琥珀金，按链号错开一点色相便于区分
		return Color(0.98, 0.66 - 0.03 * float(ci % 3), 0.20)
	# 插值链：钢蓝（与导引链对比明显）
	return Color(0.30, 0.58, 0.72)

func _update_chain_mesh() -> void:
	if _chain_mi == null or _chain_im == null or not _chain_mi.visible:
		return
	var cam := get_viewport().get_camera_3d()
	var S := _sk.global_transform
	var cam_local := S.affine_inverse() * cam.global_position if cam != null else Vector3(0.0, 1.6, 2.0)
	_chain_im.clear_surfaces()
	_chain_im.surface_begin(Mesh.PRIMITIVE_TRIANGLES)
	for ci in _chains.size():
		var ch: Dictionary = _chains[ci]
		var p: PackedVector3Array = ch["p"]
		var n := p.size()
		if n < 2:
			continue
		var base_col := _chain_tint(ci, ch["guide"])
		# 先算每个关节的左右边界（绕视线做billboard，发丝才有可见宽度）
		var left := PackedVector3Array()
		var right := PackedVector3Array()
		for j in n:
			var tan: Vector3
			if j == 0:
				tan = p[1] - p[0]
			elif j == n - 1:
				tan = p[n - 1] - p[n - 2]
			else:
				tan = p[j + 1] - p[j - 1]
			if tan.length_squared() < 1e-12:
				tan = Vector3(0.0, -1.0, 0.0)
			tan = tan.normalized()
			var to_cam := cam_local - p[j]
			var side := tan.cross(to_cam)
			if side.length_squared() < 1e-10:
				side = tan.cross(Vector3.UP)
			if side.length_squared() < 1e-10:
				side = tan.cross(Vector3.RIGHT)
			side = side.normalized()
			var w: float = chain_width * (1.0 - 0.62 * float(j) / float(n - 1))
			left.append(p[j] - side * w)
			right.append(p[j] + side * w)
		for j in n - 1:
			var c0 := base_col.darkened(0.30 * float(j) / float(n - 1))
			var c1 := base_col.darkened(0.30 * float(j + 1) / float(n - 1))
			_chain_im.surface_set_color(c0); _chain_im.surface_add_vertex(left[j])
			_chain_im.surface_set_color(c0); _chain_im.surface_add_vertex(right[j])
			_chain_im.surface_set_color(c1); _chain_im.surface_add_vertex(right[j + 1])
			_chain_im.surface_set_color(c0); _chain_im.surface_add_vertex(left[j])
			_chain_im.surface_set_color(c1); _chain_im.surface_add_vertex(right[j + 1])
			_chain_im.surface_set_color(c1); _chain_im.surface_add_vertex(left[j + 1])
		if chain_show_joints:
			for j in n:
				var tan2: Vector3 = (p[mini(j + 1, n - 1)] - p[maxi(j - 1, 0)])
				var nc := cam_local - p[j]
				var s2 := tan2.cross(nc)
				if s2.length_squared() < 1e-10:
					s2 = Vector3.RIGHT
				s2 = s2.normalized()
				var t2 := s2.cross(nc)
				t2 = t2.normalized() if t2.length_squared() > 1e-10 else Vector3.UP
				var r := chain_width * 0.9
				var jc := Color(1.0, 1.0, 1.0)
				var a0 := p[j] - s2 * r - t2 * r
				var a1 := p[j] + s2 * r - t2 * r
				var a2 := p[j] + s2 * r + t2 * r
				var a3 := p[j] - s2 * r + t2 * r
				for tri in [[a0, a1, a2], [a0, a2, a3]]:
					for vtx in tri:
						_chain_im.surface_set_color(jc)
						_chain_im.surface_add_vertex(vtx)
	_chain_im.surface_end()

# ---------------- 写回骨骼 ----------------
func _writeback() -> void:
	for c in _chains:
		var q: PackedVector3Array = c["q"]
		var p: PackedVector3Array = c["p"]
		var bones: PackedInt32Array = c["bones"]
		var n := q.size()
		for j in n:
			var u: Vector3
			var v: Vector3
			if j < n - 1:
				u = q[j + 1] - q[j]
				v = p[j + 1] - p[j]
			else:
				u = q[j] - q[j - 1]
				v = p[j] - p[j - 1]
			if u.length_squared() < 1e-12 or v.length_squared() < 1e-12:
				continue
			var uu := u.normalized()
			var vv := v.normalized()
			var rot := Quaternion(uu, vv)
			var gt := Transform3D(Basis(rot), p[j])
			_sk.set_bone_global_pose_override(bones[j], gt, 1.0, true)

# ---------------- 验收指标 ----------------
func measure() -> void:
	var max_stretch := 0.0
	for c in _guides:
		var ch: Dictionary = _chains[c]
		var p: PackedVector3Array = ch["p"]
		var lens: PackedFloat32Array = ch["len"]
		for j in range(1, p.size()):
			var l := p[j - 1].distance_to(p[j])
			if lens[j - 1] > 1e-6:
				max_stretch = maxf(max_stretch, absf(l - lens[j - 1]) / lens[j - 1])
	metric_max_stretch = max_stretch
	# 链条间最小间距
	var mn := 999.0
	for ii in _guides.size():
		var pa: PackedVector3Array = _chains[_guides[ii]]["p"]
		for jj in range(ii + 1, _guides.size()):
			var pb: PackedVector3Array = _chains[_guides[jj]]["p"]
			var n := mini(pa.size(), pb.size())
			for i in range(1, n):
				for j in range(1, n):
					if absi(i - j) > 2:
						continue
					mn = minf(mn, pa[i].distance_to(pb[j]))
	metric_min_sep = mn
	# 身体穿透
	var mp := mesh_penetration(_metric_stride)
	metric_max_pen = mp["max"]
	metric_pen_verts = mp["count"]
	metric_pen_at = mp["at"]
	metric_pen_bone = mp["bone"]

# 统计网格顶点实际用到的骨属于「导引链」还是「插值链」
func weight_usage(mi: MeshInstance3D) -> Dictionary:
	var guide_bones := {}
	var virt_bones := {}
	for ci in _chains.size():
		var is_g: bool = _chains[ci]["guide"]
		for b in _chains[ci]["bones"]:
			if is_g:
				guide_bones[b] = true
			else:
				virt_bones[b] = true
	var g := 0
	var v := 0
	var other := 0
	for surf in mi.mesh.get_surface_count():
		var a: Array = mi.mesh.surface_get_arrays(surf)
		var bo: PackedInt32Array = a[Mesh.ARRAY_BONES]
		var we: PackedFloat32Array = a[Mesh.ARRAY_WEIGHTS]
		var n: int = (a[Mesh.ARRAY_VERTEX] as PackedVector3Array).size()
		for i in n:
			var best := -1
			var bw := 0.0
			for k in 4:
				var w: float = we[i * 4 + k]
				if w > bw:
					bw = w; best = bo[i * 4 + k]
			if guide_bones.has(best):
				g += 1
			elif virt_bones.has(best):
				v += 1
			else:
				other += 1
	return {"guide": g, "virtual": v, "other": other, "bones": _sk.get_bone_count()}

# 捏人滑杆改变身体形态后：**重拟合碰撞体**并重新评估穿模候选顶点。
# 必须重拟合 —— 滑杆会改骨 pose，而头部碰撞体是按「当时的 pose 空间网格」拟合的，
# pose 一变椭球就会整体偏掉（这正是"链条又穿过脑袋"的复发原因）。
func rebuild_candidates() -> void:
	if _sk == null or not _ok:
		return
	_compute_rest_outside()
	_collect_candidates()

# 头发离身体有多远：候选顶点到身体表面的平均/最大距离（正=在体外）
func gap_stats() -> Dictionary:
	_build_shapes(0.0)
	var sum := 0.0
	var mx := 0.0
	var n := 0
	for ci in _cand_m.size():
		var mats := _bone_matrices(_hair_meshes[_cand_m[ci]])
		var vv: Vector3 = _cand_v[ci]
		var pos := Vector3.ZERO
		var ws := 0.0
		for k in 4:
			var w: float = _cand_w[ci * 4 + k]
			if w <= 0.0:
				continue
			pos += (mats[_cand_b[ci * 4 + k]] as Transform3D) * vv * w
			ws += w
		if ws <= 0.0:
			continue
		var g := -_signed_depth(pos / ws)
		sum += g
		n += 1
		mx = maxf(mx, g)
	return {"mean": sum / float(maxi(n, 1)), "max": mx, "n": n}

# 全部头发顶点到身体表面的距离统计（正=在体外多远）——衡量「头发有没有浮在身体外面」
# 诊断：每条导引链的每个关节相对头部/身体碰撞体的深度（正=在体内）
func chain_depth_report() -> Array:
	_build_shapes(0.0)
	var head_idx := -1
	for i in _colliders.size():
		if int(_colliders[i]["bone"]) == _head:
			head_idx = i
			break
	var rows := []
	for ci in _chains.size():
		var ch: Dictionary = _chains[ci]
		var p: PackedVector3Array = ch["p"]
		var worst_head := -9.9
		var worst_head_j := -1
		var worst_any := -9.9
		var worst_any_j := -1
		var inside := 0
		for j in p.size():
			if head_idx >= 0:
				var dh := _shape_signed(p[j], head_idx)
				if dh > worst_head:
					worst_head = dh
					worst_head_j = j
			var da := _signed_depth(p[j])
			if da > worst_any:
				worst_any = da
				worst_any_j = j
			if da > 0.0:
				inside += 1
		rows.append({
			"ci": ci, "guide": ch["guide"], "part": ch["part"],
			"head": worst_head, "head_j": worst_head_j,
			"any": worst_any, "any_j": worst_any_j, "inside": inside,
		})
	return rows

func gap_all(stride: int = 3) -> Dictionary:
	_build_shapes(0.0)
	var sum := 0.0
	var n := 0
	var mx := 0.0
	var mn := 1e9
	var inside := 0
	var near := 0
	for mi in _hair_meshes:
		var mats := _bone_matrices(mi)
		for surf in mi.mesh.get_surface_count():
			var a: Array = mi.mesh.surface_get_arrays(surf)
			var v: PackedVector3Array = a[Mesh.ARRAY_VERTEX]
			var bo: PackedInt32Array = a[Mesh.ARRAY_BONES]
			var we: PackedFloat32Array = a[Mesh.ARRAY_WEIGHTS]
			var i := 0
			while i < v.size():
				var acc := Vector3.ZERO
				var ws := 0.0
				for k in 4:
					var w: float = we[i * 4 + k]
					if w <= 0.0:
						continue
					acc += (mats[bo[i * 4 + k]] as Transform3D) * v[i] * w
					ws += w
				if ws > 0.0:
					var sd := _signed_depth(acc / ws)
					if sd <= 0.0:
						var g := -sd
						sum += g
						n += 1
						mx = maxf(mx, g)
						mn = minf(mn, g)
						if g < 0.030:
							near += 1
					else:
						inside += 1
				i += maxi(stride, 1)
	return {"mean_gap": sum / float(maxi(n, 1)), "max_gap": mx, "min_gap": mn,
		"near": near, "outside": n, "inside": inside}

func chain_count() -> int:
	return _chains.size()

func guide_count() -> int:
	return _guides.size()

func joint_count() -> int:
	return _chains.size() * joints_per_chain if not _chains.is_empty() else 0
func _build_shapes(margin: float) -> void:
	_sh_c.clear()
	_sh_bi.clear()
	_sh_h.clear()
	_sh_r.clear()
	for c in _colliders:
		var pc: Array = _posed_collider(c)
		var half: Vector3 = (c["half"] as Vector3) + Vector3.ONE * margin
		_sh_c.append(pc[0])
		_sh_bi.append(pc[1])
		_sh_h.append(half)
		_sh_r.append(minf(half.x, minf(half.y, half.z)))

# 静止姿态下逐顶点判断是否在身体外（1=体外，0=体内）。
# 静止时在体内的是资产本来就藏进去的部分，运动时穿出来不算穿模。
func _compute_rest_outside() -> void:
	_rest_outside.clear()
	_build_shapes(0.0)
	for mii in _hair_meshes.size():
		var mi: MeshInstance3D = _hair_meshes[mii]
		var flags := PackedByteArray()
		for surf in mi.mesh.get_surface_count():
			var a: Array = mi.mesh.surface_get_arrays(surf)
			var v: PackedVector3Array = a[Mesh.ARRAY_VERTEX]
			var bo: PackedInt32Array = a[Mesh.ARRAY_BONES]
			var we: PackedFloat32Array = a[Mesh.ARRAY_WEIGHTS]
			for i in v.size():
				var acc := Vector3.ZERO
				var ws := 0.0
				for k in 4:
					var w: float = we[i * 4 + k]
					if w <= 0.0:
						continue
					var bi: int = bo[i * 4 + k]
					if bi >= _bones.size() + 289 or bi >= _sk.get_bone_count():
						continue
					acc += (_sk.get_bone_global_rest(bi) * mi.skin.get_bind_pose(bi)) * v[i] * w
					ws += w
				var q := acc / ws if ws > 0.0 else v[i]
				var inside := false
				for si in _sh_c.size():
					var u: Vector3 = (q - _sh_c[si])
					var half: Vector3 = _sh_h[si]
					var t := Vector3(u.x / half.x, u.y / half.y, u.z / half.z)
					if t.length() < 1.0:
						inside = true
						break
				flags.append(0 if inside else 1)
		_rest_outside.append(flags)

func _push_one(pt: Vector3, si: int, r: float) -> Vector3:
	# 椭球推挤：把 pt 推到椭球面外 r 处（sd 为径向深入量，正=在体内）
	if si < 0 or si >= _sh_c.size():
		return pt
	var sd: float = _shape_signed(pt, si)
	if sd >= r:
		return pt
	var pc: Vector3 = _sh_c[si]
	var half: Vector3 = _sh_h[si]
	var bi: Basis = _sh_bi[si]
	var u := bi * (pt - pc)
	var sl := u.length()
	var dir_w: Vector3
	if sl < 1e-6:
		dir_w = Vector3(0, 1, 0)
	else:
		dir_w = (bi.transposed() * (u / sl)).normalized()
	return pt + dir_w * (r - sd)

func _bone_matrices(mi: MeshInstance3D) -> Array:
	var skin: Skin = mi.skin
	var nb: int = mini(_sk.get_bone_count(), skin.get_bind_count())
	var mats := []
	mats.resize(nb)
	for b in nb:
		mats[b] = _sk.get_bone_global_pose(b) * skin.get_bind_pose(b)
	return mats

# 链长精投影（跟随领头）：从发根（钉死）向下逐段恢复精确长度。
# 段方向取"当前链形"方向 —— 链条绕障碍缠绕时也能保持已绕开的路径。
func _solve_lengths_exact() -> void:
	for c in _guides:
		var ch: Dictionary = _chains[c]
		var p: PackedVector3Array = ch["p"]
		var lens: PackedFloat32Array = ch["len"]
		var tg: PackedVector3Array = ch["target"]
		var na: int = _attach[c] if c < _attach.size() else 1
		for j in range(1, p.size()):
			if j < na:
				p[j] = tg[j]      # 发根段钉在头皮锚点
				continue
			var d := p[j] - p[j - 1]
			var dl := d.length()
			if dl < 1e-9:
				d = Vector3(0, -1, 0)
				dl = 1.0
			p[j] = p[j - 1] + d * (lens[j - 1] / dl)
		ch["p"] = p

# 蒙皮顶点穿透（椭球度量）：供 metric_*/UI 显示。粒子级穿透见 diag_jitter。
func mesh_penetration(stride: int = 1) -> Dictionary:
	var worst := 0.0
	var count := 0
	var worst_at := Vector3.ZERO
	var worst_bone := -1
	_build_shapes(0.0)
	for mii in _hair_meshes.size():
		var mi: MeshInstance3D = _hair_meshes[mii]
		var mats := _bone_matrices(mi)
		var base := 0
		for surf in mi.mesh.get_surface_count():
			var a: Array = mi.mesh.surface_get_arrays(surf)
			var v: PackedVector3Array = a[Mesh.ARRAY_VERTEX]
			var bo: PackedInt32Array = a[Mesh.ARRAY_BONES]
			var we: PackedFloat32Array = a[Mesh.ARRAY_WEIGHTS]
			for i in v.size():
				if (base + i) % maxi(stride, 1) != 0:
					continue
				var acc := Vector3.ZERO
				var ws := 0.0
				var deep_bone := -1
				for k in 4:
					var w: float = we[i * 4 + k]
					if w <= 0.0:
						continue
					acc += (mats[bo[i * 4 + k]] as Transform3D) * v[i] * w
					ws += w
					if w > 0.4:
						deep_bone = bo[i * 4 + k]
				if ws <= 0.0:
					continue
				var q := acc / ws
				var dep: float = _signed_depth(q)
				if dep > worst:
					worst = dep
					count += 1
					worst_at = q
					worst_bone = deep_bone
	return {"max": worst, "count": count, "at": worst_at, "bone": worst_bone}
func _physics_process(delta: float) -> void:
	if not _ok or not enabled or _sk == null:
		return
	steps += 1
	var t0 := Time.get_ticks_usec()
	var dt := minf(delta, 0.033)
	_update_head(dt)
	if use_xpbd:
		_xpbd_frame(dt)
		_derive_interpolated()
		if _chain_mi != null and _chain_mi.visible:
			_update_chain_mesh()
		_writeback()
		last_solve_us = Time.get_ticks_usec() - t0
		return
	# ---- 旧 PBD 路径（备用对照，不再维护）----
	if tri_collision:
		_refresh_joint_tri()
	elif vtx_collision and _vtx_ok and steps % maxi(vtx_refresh_interval, 1) == 0:
		_refresh_joint_vtx()
	var sub: int = maxi(substeps, 1)
	var h := dt / float(sub)
	for s in sub:
		_integrate(h)
		_solve()
	var vmax := max_joint_speed * dt
	var vdamp := pow(1.0 - clampf(drag, 0.0, 0.95), dt * 60.0)
	for c in _guides:
		var ch: Dictionary = _chains[c]
		var p: PackedVector3Array = ch["p"]
		var pv: PackedVector3Array = ch["prev"]
		var j0: int = _attach[c] if c < _attach.size() else 1
		for j in range(j0, p.size()):
			var v := p[j] - pv[j]
			var vl := v.length()
			if vl > vmax:
				v = v * (vmax / vl)
			pv[j] = p[j] - v * vdamp
		ch["p"] = p
		ch["prev"] = pv
	_derive_interpolated()
	if mesh_repair and steps % maxi(mesh_repair_interval, 1) == 0:
		_mesh_repair()
	if _chain_mi != null and _chain_mi.visible:
		_update_chain_mesh()
	_writeback()

func _update_head(_dt: float) -> void:
	_head_pose = _sk.get_bone_global_pose(_head) * _head_rest_inv
	var S: Transform3D = _sk.global_transform
	_g_local = (S.basis.inverse() * Vector3(0.0, -1.0, 0.0)).normalized()
	if _first_frame:
		_first_frame = false
	else:
		# 头骨 + 骨架整体的增量，把已解出的链形带过去（保留惯性甩动感）
		var delta_m: Transform3D = (S.affine_inverse() * _S_prev) * (_head_pose * _head_pose_inv_prev)
		for c in _guides:
			var ch: Dictionary = _chains[c]
			var q: PackedVector3Array = ch["q"]
			var p: PackedVector3Array = ch["p"]
			var tg: PackedVector3Array = ch["target"]
			for j in q.size():
				tg[j] = _head_pose * q[j]
				if j > 0:
					p[j] = delta_m * p[j]
			ch["p"] = p
	_S_prev = S
	_head_pose_inv_prev = _head_pose

func _integrate(h: float) -> void:
	var k: float = lerpf(6.0, 260.0, stiffness)
	var damp: float = lerpf(0.4, 9.0, drag)
	var accel_base := _g_local * gravity + wind
	var ld := pow(linear_damping, h)
	for c in _guides:
		var ch: Dictionary = _chains[c]
		var p: PackedVector3Array = ch["p"]
		var pv: PackedVector3Array = ch["prev"]
		var tg: PackedVector3Array = ch["target"]
		var j0: int = _attach[c] if c < _attach.size() else 1
		for j in range(j0, p.size()):
			var v := (p[j] - pv[j]) * (1.0 - damp * h) * ld
			var a := (tg[j] - p[j]) * k + accel_base
			pv[j] = p[j]
			p[j] = p[j] + v + a * h * h
		ch["p"] = p
		ch["prev"] = pv

func _solve() -> void:
	for it in iterations:
		_solve_lengths()
		_solve_bending()
		if collision_enabled:
			_solve_collision_xpbd()
		_solve_boids()
	for it in 4:
		_solve_lengths_exact()
		if collision_enabled:
			_solve_collision_xpbd(true)

func _solve_lengths() -> void:
	for c in _guides:
		var ch: Dictionary = _chains[c]
		var p: PackedVector3Array = ch["p"]
		var lens: PackedFloat32Array = ch["len"]
		var na: int = _attach[c] if c < _attach.size() else 1
		for j in range(1, p.size()):
			var a := p[j - 1]
			var b := p[j]
			var d := b - a
			var dl := d.length()
			if dl < 1e-9:
				continue
			var corr := d * ((dl - lens[j - 1]) / dl)
			var fixed_a: bool = (j - 1) < na
			var fixed_b: bool = j < na
			if fixed_a and fixed_b:
				continue
			elif fixed_a:
				p[j] = b - corr
			elif fixed_b:
				p[j - 1] = a + corr
			else:
				p[j - 1] = a + corr * 0.5
				p[j] = b - corr * 0.5
		ch["p"] = p

func _solve_boids() -> void:
	if boid_separation <= 0.0:
		return
	var half := boid_separation
	var half2 := half * half
	for ii in _guides.size():
		var ca: Dictionary = _chains[_guides[ii]]
		for jj in range(ii + 1, _guides.size()):
			var cb: Dictionary = _chains[_guides[jj]]
			var pa: PackedVector3Array = ca["p"]
			var pb: PackedVector3Array = cb["p"]
			var n := mini(pa.size(), pb.size())
			var ia0: int = _attach[_guides[ii]] if _guides[ii] < _attach.size() else 1
			var ib0: int = _attach[_guides[jj]] if _guides[jj] < _attach.size() else 1
			for i in range(ia0, n):
				for j in range(maxi(ib0, i - 2), mini(n - 1, i + 2) + 1):
					var d := pb[j] - pa[i]
					var l2 := d.length_squared()
					if l2 >= half2 or l2 < 1e-12:
						continue
					var pushv := d * ((half - sqrt(l2)) / sqrt(l2)) * 0.5 * boid_strength
					pa[i] = pa[i] - pushv
					pb[j] = pb[j] + pushv

func _solve_bending() -> void:
	if bend_stiffness <= 0.0:
		return
	for c in _guides:
		var ch: Dictionary = _chains[c]
		var p: PackedVector3Array = ch["p"]
		var bend: PackedFloat32Array = ch["bend"]
		var na: int = _attach[c] if c < _attach.size() else 1
		for j in range(1, p.size() - 1):
			if bend[j] <= 0.0:
				continue
			var a := p[j - 1]
			var cc := p[j + 1]
			var d := cc - a
			var dl := d.length()
			if dl < 1e-9:
				continue
			var corr := d * ((dl - bend[j]) / dl) * 0.5 * bend_stiffness
			if j - 1 >= na:
				p[j - 1] = a + corr
			if j + 1 >= na:
				p[j + 1] = cc - corr
		ch["p"] = p

