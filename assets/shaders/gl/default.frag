uniform sampler2D s_texture;
uniform vec4 color;
uniform vec4 texMult;

varying vec2 texCoord;
varying vec3 normal;
uniform mat3 normalMatrix;

void main()
{
	vec4 texColor = texMult * texture2D(s_texture, texCoord);

	if(color.a + texColor.a <= 0.01)
		discard;

	float diffuseFac = abs(dot(normalize(normal * normalMatrix), normalize(vec3(1,1,1))));


	gl_FragColor = mix(color + texColor, (color + texColor) * vec4(diffuseFac,diffuseFac,diffuseFac,1), 1.0);
}