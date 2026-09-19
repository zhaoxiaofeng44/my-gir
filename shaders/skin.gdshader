// 皮肤着色器 —— 参考 HumanShaders 思路
// 次表面散射 (sss_mode_skin 增强红通道) + 菲涅尔边缘光 + 可选 Normal/Detail 贴图
shader_type spatial;
render_mode blend_mix, depth_draw_opaque, cull_back, diffuse_burley, specular_schlick_ggx, sss_mode_skin;

uniform vec4 base_color : source_color = vec4(0.95, 0.78, 0.70, 1.0);
uniform sampler2D albedo_texture : source_color, hint_default_white;
uniform sampler2D normal_texture : hint_normal;
uniform bool use_normal_map = false;
uniform float normal_strength : hint_range(0.0, 2.0) = 0.6;
uniform float roughness : hint_range(0.0, 1.0) = 0.52;
uniform float specular_amount : hint_range(0.0, 1.0) = 0.4;
// 次表面散射
uniform float subsurface_strength : hint_range(0.0, 4.0) = 1.4;
// 皮肤油脂感的高光贴图（可选）
uniform sampler2D roughness_texture : hint_default_white;
// 微弱红色边缘(耳朵/鼻翼透光感)
uniform float rim_intensity : hint_range(0.0, 1.0) = 0.12;
uniform vec4 rim_color : source_color = vec4(0.9, 0.35, 0.25, 1.0);

void fragment() {
	vec4 tex = texture(albedo_texture, UV);
	vec3 albedo = base_color.rgb * tex.rgb;
	// 红通道轻微提亮：抵消蜡质感
	albedo.r = pow(clamp(albedo.r, 0.0, 1.0), 0.96);
	ALBEDO = albedo;
	ROUGHNESS = roughness * texture(roughness_texture, UV).r;
	SPECULAR = specular_amount;
	if (use_normal_map) {
		NORMAL_MAP = texture(normal_texture, UV).rgb;
		NORMAL_MAP_DEPTH = normal_strength;
	}
	// Godot 内置 SSS：皮肤模式会加权红通道
	SSS_STRENGTH = subsurface_strength;
	// 边缘透光的假 SSS（耳廓、指尖）
	float fresnel = pow(1.0 - clamp(dot(NORMAL, VIEW), 0.0, 1.0), 3.0);
	BACKLIGHT = rim_color.rgb * rim_intensity * fresnel * 4.0;
}
