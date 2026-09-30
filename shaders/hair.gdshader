// hair.gdshader —— 真实感头发：Kajiya-Kay 双叶各向异性高光 + 根梢渐变 + 根部遮蔽 + 透光 + 发梢柔化
//
// 与旧版的区别：
//   ① 发丝切线不再靠屏幕空间导数(dFdx/dFdy)猜，而是**建模时按发链方向逐顶点烘焙进 TANGENT**，
//      因此 UV 布局怎么排都不影响（HS2 的发片 UV 并不沿发丝走）。
//   ② 逐顶点烘焙 COLOR.r = 沿发链的归一化参数(0=发根,1=发梢)、COLOR.g = 朝内发片的遮蔽系数，
//      这样根梢渐变和假 AO 与资产无关，换发模也成立。
//   ③ 双叶高光（主高光 + 次级偏移高光，动漫头发标志性的两道高光带）。
//   ④ 透光(backlight) + 包裹漫反射，逆光时发丝边缘会透亮。
//   ⑤ 双面渲染时翻转法线，背面受光才正确；发梢自动淡化，避免"被剪断"的硬边。
shader_type spatial;
render_mode blend_mix, depth_draw_opaque, cull_disabled, specular_schlick_ggx, diffuse_burley;

uniform sampler2D albedo_texture : source_color, hint_default_white;
uniform vec4 hair_tint : source_color = vec4(0.72, 0.55, 0.40, 1.0);

// —— 根梢渐变（注意：这两个是**乘性因子**，不是颜色；>1 表示提亮，不加 source_color 以免被当 sRGB 转换）——
uniform vec4 root_color = vec4(0.50, 0.46, 0.44, 1.0);
uniform vec4 tip_color = vec4(1.08, 1.02, 0.96, 1.0);
uniform float grad_curve : hint_range(0.2, 4.0) = 1.5;   // >1：发根保持深色更久，只让梢部变亮

// —— 裁剪 ——
uniform float alpha_scissor : hint_range(0.0, 1.0) = 0.35;
uniform float tip_fade : hint_range(0.0, 1.0) = 0.18;

// —— 假 AO ——
uniform float root_ao : hint_range(0.0, 1.0) = 0.55;
uniform float root_ao_len : hint_range(0.02, 1.0) = 0.30;
uniform float inner_ao : hint_range(0.0, 1.0) = 0.45;

// —— 各向异性高光（Kajiya-Kay 双叶）——
uniform float primary_shift : hint_range(-2.0, 2.0) = 0.12;
uniform float primary_power : hint_range(8.0, 512.0) = 140.0;
uniform float primary_gain : hint_range(0.0, 3.0) = 0.08;
uniform float secondary_shift : hint_range(-2.0, 2.0) = -0.28;
uniform float secondary_power : hint_range(8.0, 512.0) = 26.0;
uniform float secondary_gain : hint_range(0.0, 3.0) = 0.04;
uniform vec4 secondary_color : source_color = vec4(0.85, 0.72, 0.55, 1.0);

// —— 透光 / 边缘 ——
uniform float translucency : hint_range(0.0, 3.0) = 0.25;
uniform vec4 translucency_color : source_color = vec4(0.78, 0.46, 0.28, 1.0);
uniform float rim : hint_range(0.0, 1.0) = 0.10;

uniform float roughness : hint_range(0.0, 1.0) = 0.45;
uniform float specular_amount : hint_range(0.0, 1.0) = 0.30;

varying vec3 strand_tan;
varying float strand_t;
varying float inner_term;

void vertex() {
	// TANGENT 是建模阶段按发链方向烘焙的（HS2 贴片头发资产自带；物理头发不走这个着色器）
	strand_tan = (MODEL_MATRIX * vec4(TANGENT, 0.0)).xyz;
	strand_t = COLOR.r;
	inner_term = COLOR.g;
}

void fragment() {
	vec4 tex = texture(albedo_texture, UV);
	float a = tex.a;
	// 发梢柔化：越靠梢部越容易裁掉，得到渐细的末端而不是一刀切
	a *= 1.0 - tip_fade * smoothstep(0.70, 1.0, strand_t);
	if (a < alpha_scissor) {
		discard;
	}
	// 背面翻转法线（本材质 cull_disabled，两面都要正确受光）
	if (!FRONT_FACING) {
		NORMAL = -NORMAL;
	}
	// 根梢渐变 × 根部压暗 × 朝内发片遮蔽
	float gt = pow(clamp(strand_t, 0.0, 1.0), grad_curve);
	vec3 grad = mix(root_color.rgb, tip_color.rgb, gt);
	float occ = mix(1.0 - root_ao, 1.0, smoothstep(0.0, root_ao_len, strand_t));
	occ *= mix(1.0, 1.0 - inner_ao, clamp(inner_term, 0.0, 1.0));
	ALBEDO = tex.rgb * hair_tint.rgb * grad * occ;
	ROUGHNESS = roughness;
	SPECULAR = specular_amount;
	// 轻微边缘丝光
	float fres = pow(1.0 - clamp(dot(normalize(NORMAL), normalize(VIEW)), 0.0, 1.0), 3.0);
	EMISSION = grad * fres * rim * 0.20;
}

void light() {
	vec3 T = normalize(strand_tan);
	float TdotL = dot(T, LIGHT);
	float sinTL = sqrt(max(0.0, 1.0 - TdotL * TdotL));
	float s1 = TdotL + primary_shift;
	float s2 = TdotL + secondary_shift;
	float lobe1 = pow(clamp(sinTL * sqrt(max(0.0, 1.0 - s1 * s1)), 0.0, 1.0), primary_power);
	float lobe2 = pow(clamp(sinTL * sqrt(max(0.0, 1.0 - s2 * s2)), 0.0, 1.0), secondary_power);
	// 注意：LIGHT_COLOR 已经含灯光能量，增益必须小，否则整片头发会被高光冲成白色
	SPECULAR_LIGHT += (lobe1 * primary_gain * LIGHT_COLOR + lobe2 * secondary_gain * secondary_color.rgb * LIGHT_COLOR) * ATTENUATION;
	// 包裹漫反射：只把明暗交界稍微推开一点（推太多会整体发白）
	float wrap = clamp((dot(NORMAL, LIGHT) + 0.18) / 1.18, 0.0, 1.0);
	// 透光：光从背面穿过发丝
	float trans = pow(clamp(-dot(NORMAL, LIGHT), 0.0, 1.0), 1.4);
	vec3 diff = ALBEDO * LIGHT_COLOR * (wrap + trans * translucency * translucency_color.rgb);
	DIFFUSE_LIGHT += diff * ATTENUATION;
}
