// 布料着色器 —— 顶点风动画 + Sheen 边缘光 + 折痕 AO
// 轻量替代 GPU Cloth Simulation 物理方案（见 README）
shader_type spatial;
render_mode blend_mix, depth_draw_opaque, cull_back, diffuse_burley, specular_schlick_ggx;

uniform sampler2D cloth_texture : source_color, hint_default_white;
uniform vec4 base_color : source_color = vec4(0.35, 0.12, 0.18, 1.0);
uniform float roughness : hint_range(0.0, 1.0) = 0.85;
uniform float sheen_intensity : hint_range(0.0, 2.0) = 0.6;
uniform vec4 sheen_color : source_color = vec4(0.9, 0.8, 0.75, 1.0);
// 风力顶点动画
uniform float wind_amplitude = 0.03;
uniform float wind_frequency : hint_range(0.0, 20.0) = 3.0;
uniform float wind_speed = 1.5;
uniform float wave_scale = 6.0;
uniform float sway_strength = 0.5; // 顶部摆动(UV.y=1) 底部固定(UV.y=0)

varying float fold_ao;

void vertex() {
	float t = TIME * wind_speed;
	float sway = UV.y * sway_strength;
	float wave = sin(VERTEX.x * wave_scale + t) + 0.5 * sin(VERTEX.y * wave_scale * 1.7 + t * 1.3);
	VERTEX += NORMAL * wave * wind_amplitude * sway;
	fold_ao = 0.75 + 0.25 * sin(VERTEX.x * wave_scale * 2.0);
}

void fragment() {
	vec3 albedo = base_color.rgb * texture(cloth_texture, UV).rgb;
	ALBEDO = albedo * fold_ao;
	ROUGHNESS = roughness;
	SPECULAR = 0.15;
	// 织物 Sheen：掠射角出现柔和反光
	float f = pow(1.0 - clamp(dot(NORMAL, VIEW), 0.0, 1.0), 3.5);
	RIM = sheen_intensity * f * 0.5;
	RIM_TINT = 0.8;
}
