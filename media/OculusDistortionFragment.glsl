
uniform sampler2D videoStream;
uniform sampler2D diffuseMap;

void main(void)
{
	float red = texture2D(videoStream, gl_TexCoord[0].xy).r;
	float green = texture2D(videoStream, gl_TexCoord[1].xy).g;
	float blue = texture2D(videoStream, gl_TexCoord[2].xy).b;
	float alpha = texture2D(videoStream, gl_TexCoord[2].xy).a;

	vec4 colorVideo = vec4( red, green, blue, 1.0 );

	red = texture2D(diffuseMap, gl_TexCoord[0].xy).r;
	green = texture2D(diffuseMap, gl_TexCoord[1].xy).g;
	blue = texture2D(diffuseMap, gl_TexCoord[2].xy).b;
	alpha = texture2D(diffuseMap, gl_TexCoord[2].xy).a;

	//vec4 colorScene = vec4( red, green, blue, 1.0 )*gl_Color;
	vec4 colorScene = vec4( red, green, blue, 1.0 );

	red = min( colorVideo.r, colorScene.r );
	green = min( colorVideo.g, colorScene.g );
	blue = min( colorVideo.b, colorScene.b );

	gl_FragColor = colorVideo + colorScene;

	gl_FragColor = gl_FragColor*gl_Color;

	//gl_FragColor = texture2D(diffuseMap, gl_TexCoord[0].xy);//vec4( red, green, blue, 1.0 );
	//gl_FragColor = vec4( 1.0, 1.0, 1.0, 1.0 );
}
