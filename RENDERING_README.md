# 电影级人物渲染套件 (Cinematic Character Rendering Kit)

本工程内置一套自包含的人物渲染着色器 + 演示场景 `character_demo.tscn`。

## 内置着色器

| 文件 | 技术 |
| --- | --- |
| `shaders/skin.gdshader` | `sss_mode_skin` 次表面散射(红通道加权)、Burley 漫反射、假 SSS 背光(BACKLIGHT)、边缘红润 |
| `shaders/eye.gdshader` | 程序化虹膜/瞳孔、视线视差模拟虹膜凹陷、角膜湿润高光、角膜缘环(limbal ring) |
| `shaders/hair.gdshader` | Kajiya-Kay 双叶各向异性高光、Alpha 裁剪、根梢渐变、透光、可选 Flow Map |
| `shaders/cloth.gdshader` | 顶点风场动画、织物 Sheen、折痕 AO（轻量布料方案） |

## 使用方法
1. 在 Godot 编辑器中打开 `character_demo.tscn`，直接 F5/F6 预览（AgX 色调映射已启用）。
2. 把对应 `.gdshader` 拖到你的角色 MeshInstance3D 的 Surface Material Override 上。
3. 接入真实模型后，把 Albedo/Alpha/Normal/Flow 贴图填入 uniform 即可。

## 推荐接入的外部开源插件（自行下载安装）
- **HumanShaders** (MatMADNESS)：更完整的皮肤/眼球/湿润度着色器 + AgX 工作流 — https://github.com/matjad/HumanShaders
- **Hair!** (Pelatho)：Kajiya-Kay 头发着色器工程 — https://github.com/Pelatho/GodotHair
- **TressFX Godot 移植**：GPU 发丝模拟 + SDF 碰撞 (GDExtension) — 搜索 "TressFX godot gdextension"
- **GPU Cloth Simulation**：计算着色器 PBD 布料；**Godot-Silkload**：Verlet 骨骼布料 (Godot 4.4+)
- 安装方式：插件目录放入 `res://addons/`，然后在 项目设置 → 插件 中启用。

## Intel Mac (Intel Iris + MoltenVK) 注意事项
- 崩溃特征：macOS 崩溃报告落在 IGAccelRenderCommandEncoder::programPipeline，并伴随 "VK_ERROR_INITIALIZATION_FAILED: Compute pipeline compile failed"。
- 原因：Forward+ 依赖大量 compute 管线（SSAO/Glow/SSIL 等），Intel Iris 655 的 MoltenVK 驱动实现不完整；此外在 light() 中使用屏幕空间导数(dFdx/dFdy)也会触发驱动崩溃——本项目头发着色器已改为在 fragment() 中计算切线。
- 本项目已默认关闭 SSAO/Glow、使用 Filmic 色调映射。若仍崩溃，编辑 project.godot 将 renderer/rendering_method 改为 "gl_compatibility"（注意：兼容模式下 Godot 不支持 SSS，皮肤着色器的 SSS_STRENGTH 会被忽略，仅剩 BACKLIGHT 假 SSS）。
- 在 Apple Silicon 或独显机器上可恢复 AgX(tonemap_mode = 4) 与 SSAO/Glow 获得完整效果。

## 性能要点
- LOD：项目设置 `Rendering > Mesh LOD > Threshold Pixels`；导入模型时启用 generate LOD。
- Godot 4 默认 GPU 蒙皮；骨骼预算按目标平台规划（移动端 ≤ 75 骨骼/角色）。
- 头发 Alpha 裁剪优于透明混合（无排序问题）；覆盖层(湿感)才用 alpha blend。

## 版本管理（git）

本地仓库已初始化，首次提交为 `feat: HS2 捏人系统迁移到 Godot + 电影级人物渲染套件`。

```bash
git log --oneline          # 查看历史
git status                 # 查看改动
git diff                   # 查看未暂存改动
```

常用文件（提交范围）：
| 路径 | 内容 |
| --- | --- |
| `hs2/` | 捏人数据层 / 骨架合并 / 角色装配 / 轨道相机 / 原版数据与资产 |
| `shaders/` | 皮肤·眼睛·头发·布料 四个着色器 |
| `tools/` | `fbx2glb.py`(FBX→glb)、`acceptance.gd`(机器验收)、`blender_oracle.py`(期望值) |
| `*.tscn` | 四个演示场景 |

`.gitignore` 排除的内容（如需恢复）：
- `.godot/`、`.dsh-vision-router/` — 缓存/工具产物，删掉后会重建
- `addons/TressFX-Godot/` — 349M，需自行 `git clone https://github.com/ismailivanov/TressFX-Godot.git` 并编译 GDExtension
- `addons/HumanShaders/Models/{Monster,HumanHead/EmilyHeadScan}/` — 未使用的大模型，
  可从 `https://github.com/matmadness/HumanShaders` 重新获取
- `hs2/model/{face_n.TGA, cf_s_body_00_00_00-RGB24.png, Animation/}` — 未被引用的原始资产，
  在原版 `HoneySelect/Assets/Cosmetic/` 下

> 本仓库含约 600MB 二进制资产（HS2 模型/贴图、VRM、高模头）。
> 若以后要推送到远端，建议先接 Git LFS：`git lfs track "*.glb" "*.png" "*.vrm" "*.TGA"`。
> 提交身份为仓库局部配置（`myGirl Dev <dev@local>`），需要改成自己的可执行
> `git config user.name "你的名字" && git config user.email "你的邮箱"`。
