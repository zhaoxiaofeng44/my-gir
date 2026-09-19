// 眼睛着色器 —— 程序化虹膜 + 角膜折射视差 + 湿润高光
// 参考 HumanShaders 眼球着色器 与 Refractive Procedural Eye
shader_type spatial;
render_mode blend_mix, depth_draw_opaque, cull_back, diffuse_burley, specular_schlick_ggx;

uniform vec4 sclera_color : source_color = vec4(0.93, 0.90, 0.87, 1.0);
uniform vec4 iris_color : source_color = vec4(0.22, 0.42, 0.55, 1.0);
uniform vec4 limbal_color : source_color = vec4(0.05, 0.05, 0.07, 1.0);
uniform float iris_size : hint_range(0.2, 0.95) = 0.62;     // 虹膜相对眼球半径
uniform float pupil_dilation : hint_range(0.05, 0.6) = 0.25; // 瞳孔相对虹膜
uniform float iris_depth : hint_range(0.0, 0.5) = 0.18;      // 虹膜凹陷视差强度
uniform float cornea_gloss : hint_range(0.0, 1.0) = 1.0;     // 湿润感
uniform float sclera_redness : hint_range(0.0, 1.0) = 0.15;  // 眼角血丝红

varying vec3 local_n;

void vertex() {
	local_n = normalize(VERTEX); // 单位球局部坐标：+Z 为瞳孔朝向
}

void fragment() {
	vec3 n = normalize(local_n);
	// 用视线方向做简单折射视差，模拟虹膜在角膜下的深度
	vec2 shift = -normalize(VIEW).xy * iris_depth * 0.35;
	vec2 p = n.xy + shift;
	float r = length(p) / max(iris_size, 0.001);

	vec4 col = sclera_color;
	// 眼角微微泛红
	col = mix(col, vec4(0.85, 0.55, 0.5, 1.0), sclera_redness * smoothstep(0.6, 1.0, abs(p.x) / max(iris_size, 0.001)));

	if (r < 1.0) {
		// 虹膜：径向纹理 + 瞳孔
		float ang = atan(p.y, p.x);
		float fibers = 0.5 + 0.5 * sin(ang * 42.0 + r * 9.0);
		float fibers2 = 0.5 + 0.5 * sin(ang * 17.0 - r * 22.0);
		vec3 iris = iris_color.rgb * (0.65 + 0.35 * fibers * fibers2);
		iris *= mix(0.6, 1.15, smoothstep(1.0, 0.25, r)); // 外缘偏暗
		float pupil = smoothstep(pupil_dilation, pupil_dilation * 0.75, r);
		iris = mix(iris, vec3(0.01), pupil);
		// 角膜在虹膜上的反光让虹膜“活”起来
		float caustic = pow(max(dot(normalize(-VIEW + vec3(0.3, 0.5, 1.0)), n), 0.0), 8.0);
		iris += caustic * 0.08;
		col = mix(col, vec4(iris, 1.0), smoothstep(1.02, 0.95, r));
	}
	// 角膜缘环（limbal ring）
	float limbal = smoothstep(0.82, 1.0, r) * smoothstep(1.12, 1.0, r);
	col = mix(col, limbal_color, limbal * 0.85);

	ALBEDO = col.rgb;
	ROUGHNESS = 0.03;
	SPECULAR = 0.2 + cornea_gloss * 0.7;
	METALLIC = 0.0;
	// 湿润菲涅尔
	float f = pow(1.0 - clamp(dot(NORMAL, VIEW), 0.0, 1.0), 4.0);
	RIM = cornea_gloss * f;
	RIM_TINT = 0.5;
}
