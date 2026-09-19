// 头发着色器 —— Kajiya-Kay 双叶各向异性高光 + Alpha 裁剪 + 透光 + 根梢渐变
shader_type spatial;
render_mode blend_mix, depth_draw_opaque, cull_disabled, specular_schlick_ggx;

uniform sampler2D hair_texture : source_color, hint_default_white;
uniform sampler2D alpha_texture : hint_default_white;
uniform float alpha_scissor : hint_range(0.0, 1.0) = 0.35;
uniform vec4 root_color : source_color = vec4(0.12, 0.07, 0.05, 1.0);
uniform vec4 tip_color : source_color = vec4(0.55, 0.38, 0.22, 1.0);
uniform float root_tip_blend = 1.0;
uniform float primary_shift : hint_range(-2.0, 2.0) = 0.10;
uniform float primary_power : hint_range(8.0, 512.0) = 120.0;
uniform float primary_gain : hint_range(0.0, 2.0) = 0.55;
uniform float secondary_shift : hint_range(-2.0, 2.0) = -0.25;
uniform float secondary_power : hint_range(8.0, 512.0) = 28.0;
uniform float secondary_gain : hint_range(0.0, 2.0) = 0.45;
uniform vec4 secondary_color : source_color = vec4(0.7, 0.55, 0.4, 1.0);
uniform float translucency : hint_range(0.0, 2.0) = 0.5;
uniform vec4 translucency_color : source_color = vec4(0.65, 0.45, 0.3, 1.0);
uniform sampler2D flow_map : hint_default_black;
uniform float flow_strength : hint_range(0.0, 1.0) = 0.0;
uniform float roughness : hint_range(0.0, 1.0) = 0.35;
uniform float trim_bottom = -2.0;
uniform float trim_front = 2.0;
uniform float normal_expand = 0.0;
uniform bool scalp_mode = false;
uniform float aabb_min_y = 0.0;
uniform float aabb_inv_h = 0.01;
uniform float aabb_min_z = 0.0;
uniform float aabb_inv_d = 0.01;
uniform float hairline = 0.70;
uniform float front_limit = 0.58;

varying vec3 world_pos;
varying vec2 hair_uv;
varying vec3 tan_ws;
varying vec3 obj_pos;

void vertex() {
	world_pos = (MODEL_MATRIX * vec4(VERTEX, 1.0)).xyz;
	hair_uv = UV;
	obj_pos = VERTEX;
	VERTEX += NORMAL * normal_expand;
}

// 屏幕空间导数重建发丝切线（必须在 fragment 中调用，勿放 light()）
vec3 compute_tangent() {
	vec3 dp1 = dFdx(world_pos);
	vec3 dp2 = dFdy(world_pos);
	vec2 duv1 = dFdx(hair_uv);
	vec2 duv2 = dFdy(hair_uv);
	vec3 t = normalize(duv2.y * dp1 - duv1.y * dp2);
	vec2 flow = texture(flow_map, hair_uv).rg * 2.0 - 1.0;
	return normalize(t + vec3(flow.x, 0.0, flow.y) * flow_strength * 0.3);
}

void fragment() {
	vec4 tex = texture(hair_texture, hair_uv);
	float a = tex.a * texture(alpha_texture, hair_uv).a;
	if (a < alpha_scissor) { discard; }
	if (obj_pos.y < trim_bottom || obj_pos.z > trim_front) { discard; }
	if (scalp_mode) {
		float h = (world_pos.y - aabb_min_y) * aabb_inv_h;
		float zn = (world_pos.z - aabb_min_z) * aabb_inv_d;
		if (h < hairline || zn > front_limit) { discard; }
	}
	tan_ws = compute_tangent();
	ALBEDO = mix(root_color.rgb, tex.rgb * tip_color.rgb, clamp(hair_uv.y * root_tip_blend, 0.0, 1.0));
	ROUGHNESS = roughness;
	SPECULAR = 0.4;
}

void light() {
	vec3 T = normalize(tan_ws);
	float TdotL = dot(T, LIGHT);
	float TdotE = dot(T, -VIEW);
	float sinTL = sqrt(max(0.0, 1.0 - TdotL * TdotL));
	float shift1 = TdotL + primary_shift;
	float shift2 = TdotL + secondary_shift;
	float spec1 = primary_gain * pow(clamp(sinTL * sqrt(max(0.0, 1.0 - shift1 * shift1)), 0.0, 1.0), primary_power);
	float spec2 = secondary_gain * pow(clamp(sinTL * sqrt(max(0.0, 1.0 - shift2 * shift2)), 0.0, 1.0), secondary_power);
	SPECULAR_LIGHT += (spec1 * LIGHT_COLOR + spec2 * secondary_color.rgb) * ATTENUATION;
	float wrap = clamp((dot(NORMAL, LIGHT) + 0.4) / 1.4, 0.0, 1.0);
	float trans = clamp(-dot(NORMAL, LIGHT), 0.0, 1.0);
	vec3 diff = ALBEDO * LIGHT_COLOR * (wrap + trans * translucency * translucency_color.rgb);
	DIFFUSE_LIGHT += diff * ATTENUATION;
}