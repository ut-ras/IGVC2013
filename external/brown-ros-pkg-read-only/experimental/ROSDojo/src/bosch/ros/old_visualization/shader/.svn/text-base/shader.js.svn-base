/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Robert Bosch LLC.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Robert Bosch nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

ros.visualization.ShaderManager = function (gl) {
	this.ShaderTypes = {"SIMPLE" : 0, "FLAT" : 1, "PHONG" : 2, "TEXTURE" : 3, "SIMPLE_COLOR" : 4,"POINT_CLOUD" : 5,"TEXTURE_TEXT" : 6};
  
  this.shaderPrograms = [];
  
  var simpleVertexShaderSource = [
                                  "#ifdef GL_ES",
                                  "precision highp float;",
                                  "#endif",
                                  "", 
                                  "uniform   mat4 u_mvp;",
                                  "uniform   vec3 u_color;",
                                  "", 
                                  "attribute vec3 a_position;",
                                  "",
                                  "varying   vec3 v_color;",
                                  "",
                                  "void main(void)",
                                  "{",
                                  "  v_color    = u_color;",
                                  "  gl_Position = u_mvp * vec4(a_position, 1.0);",
                                  "}"
                                 ].join("\n");
  
  var simpleFragmentShaderSource = [
                                    "#ifdef GL_ES",
                                    "precision highp float;",
                                    "#endif",
                                    "", 
                                    "varying vec3 v_color;",
                                    "",
                                    "void main(void)",
                                    "{",
                                    "  gl_FragData[0] = vec4(v_color, 1.0);",
                                    "}"
                                    ].join("\n");
 
  var simpleColorVertexShaderSource = [
                                  "#ifdef GL_ES",
                                  "precision highp float;",
                                  "#endif",
                                  "", 
                                  "uniform   mat4 u_mvp;",
                                  "", 
                                  "attribute vec3 a_position;",
                                  "attribute vec3 a_color;",
                                  "",
                                  "varying   vec3 v_color;",
                                  "",
                                  "void main(void)",
                                  "{",
                                  "  v_color    = a_color;",
                                  "  gl_Position = u_mvp * vec4(a_position, 1.0);",
                                  "}"
                                 ].join("\n");
  
  var simpleColorFragmentShaderSource = [
                                    "#ifdef GL_ES",
                                    "precision highp float;",
                                    "#endif",
                                    "", 
                                    "varying vec3 v_color;",
                                    "",
                                    "void main(void)",
                                    "{",
                                    "  gl_FragData[0] = vec4(v_color, 1.0);",
                                    "}"
                                    ].join("\n");
  
  var pointCloudVertexShaderSource = [
                                      "#ifdef GL_ES",
                                      "precision highp float;",
                                      "#endif",
                                      "", 
                                      "uniform   mat4 u_mvp;",
                                      "uniform   mat4 u_mv;",
                                      "uniform float u_pointsize;",
                                      "uniform vec3 u_attenuation;",
                                      "",
                                      "varying vec3 v_color;",
                                      "", 
                                      "attribute vec3 a_position;",
                                      "attribute vec3 a_color;",
                                      "",
                                      "void main(void)",
                                      "{",
                                      "  v_color = a_color;",
                                      "  vec4 ecPos4 = u_mv * vec4(a_position, 1.0);",
                                      "  float dist = length(ecPos4);",
                                      "  float attn = u_attenuation[0] + (u_attenuation[1] * dist) +  (u_attenuation[2] * dist * dist);",
                                      "",
                                      "  gl_Position = u_mvp * vec4(a_position, 1.0);",
                                      "  gl_PointSize = (attn > 0.0) ? u_pointsize * sqrt(1.0/attn) : 1.0;",
                                      "}"
                                     ].join("\n");
      
   var pointCloudFragmentShaderSource = [
                                        "#ifdef GL_ES",
                                        "precision highp float;",
                                        "#endif",
                                        "", 
                                        "varying vec3 v_color;",
                                        "",
                                        "void main(void)",
                                        "{",
                                        "  gl_FragColor = vec4(v_color, 1.0);",
                                        "}"
                                        ].join("\n");
      
  var flatVertexShaderSource = [
                                  "#ifdef GL_ES",
                                  "precision highp float;",
                                  "#endif",
                                  "", 
                                  "uniform   mat4 u_mvp;",
                                  "uniform   mat3 u_view_normal_mat;",
                                  "uniform   vec3 u_color;",
                                  "",
                                  "attribute vec3 a_position;",
                                  "attribute vec3 a_normal;",
                                  "",
                                  "varying   vec3 v_color;",
                                  "varying   vec3 v_normal;",
                                  "",
                                  "void main(void)",
                                  "{",
                                  "  v_color     = u_color;",
                                  "  v_normal    = u_view_normal_mat * a_normal;",
                                  "  gl_Position = u_mvp * vec4(a_position, 1.0);",
                                  "}"
                                  ].join("\n");
  
  var flatFragmentShaderSource = [
                                  "#ifdef GL_ES",
                                  "precision highp float;",
                                  "#endif",
                                  "", 
                                  "varying vec3 v_color;",
                                  "varying vec3 v_normal;",
                                  "void main(void)",
                                  "{",
                                  "  const vec3 lightDir = vec3(0.0, 0.0, -1.0);",
                                  "  vec3  normal  = normalize(v_normal);",
                                  "  float lambert = max(dot(normal, -lightDir), 0.0);",
                                  "  vec3  color   = v_color * lambert;",
                                  "  gl_FragColor  = vec4(color, 1.0);",
                                  "}"
                                  ].join("\n");
  
  var phongVertexShaderSource = [ 
                                  "#ifdef GL_ES",
                                  "precision highp float;",
                                  "#endif",
                                  "", 
                                  "uniform   mat4 u_model_view_projection_mat;",
                                  "uniform   mat4 u_model_view_mat;",
                                  "uniform   mat3 u_view_normal_mat;",
                                  "",
                                  "attribute vec4 a_position;",
                                  "attribute vec3 a_normal;",
                                  "attribute vec2 a_texcoord;",
                                  "",
                                  "varying   vec3 v_view_position;",
                                  "varying   vec3 v_view_normal;",
                                  "varying   vec2 v_texcoord;",
                                  "",
                                  "void main(void)",
                                  "{",
                                  "", 
                                  "  v_view_position = (u_model_view_mat  * a_position).xyz;",
                                  "  v_view_normal   = u_view_normal_mat * a_normal;",
                                  "  v_texcoord      = a_texcoord;",
                                  "",
                                  "  gl_Position = u_model_view_projection_mat * a_position;",
                                  "}"
                                  ].join("\n");
  
  var phongFragmentShaderSource = [
                                    "#ifdef GL_ES",
                                    "precision highp float;",
                                    "#endif",
                                    "", 
                                    "uniform vec4      u_view_light_dir;",
                                    "",
                                    "uniform vec4      u_emission;",
                                    "uniform vec4      u_ambient;",
                                    "uniform vec4      u_diffuse;",
                                    "uniform vec4      u_specular;",
                                    "uniform float     u_shininess;",
                                    " ",
                                    "uniform sampler2D s_emission;",
                                    "uniform sampler2D s_ambient;",
                                    "uniform sampler2D s_diffuse;",
                                    "uniform sampler2D s_specular;",
                                    "uniform sampler2D s_shininess;",
                                    " ",
                                    "varying vec3      v_view_position;",
                                    "varying vec3      v_view_normal;",
                                    "varying vec2      v_texcoord;",
                                    " ",
                                    "void main(void)",
                                    "{",
                                    "  vec3  matEmission  = u_emission.xyz + texture2D(s_emission, v_texcoord).xyz;",
                                    "  vec3  matAmbient   = u_ambient.xyz;",
                                    "  vec3  matDiffuse   = u_diffuse.xyz * texture2D(s_diffuse, v_texcoord).xyz;",
                                    "  vec3  matAccessibility = texture2D(s_ambient, v_texcoord).xyz;",
                                    "  vec3  matSpecular  = u_specular.xyz * texture2D(s_specular, v_texcoord).xyz;",
                                    "  float matShininess = u_shininess + texture2D(s_shininess, v_texcoord).x;",
                                    " ",
                                    "  vec3  view_normal  = normalize(v_view_normal);",
                                    "  float n_dot_l      = abs(dot(view_normal, -u_view_light_dir.xyz));",
                                    "  if (u_view_light_dir.w > 0.5) n_dot_l = 1.0;",
                                    "  vec3  refl_vector  = reflect(u_view_light_dir.xyz, view_normal);",
                                    "  vec3  view_vector  = normalize(-v_view_position);",
                                    " ",
                                    "  vec3  emission = matEmission;",
                                    "  vec3  ambient  = matAmbient * matDiffuse;",
                                    "  vec3  diffuse  = matDiffuse * n_dot_l * matAccessibility;",
                                    " vec3  specular = matSpecular * pow(max(dot(refl_vector, view_vector), 0.0), u_shininess);",
                                    " ",
                                    "  vec3  color = emission + ambient + diffuse + specular;",
                                    " ",
                                    "  gl_FragData[0] = vec4(color, 1.0);",
                                    "}"
                                     ].join("\n");
  
  var textureVertexShaderSource = [
                                   "#ifdef GL_ES",
                                   "precision highp float;",
                                   "#endif",
                                   "",  
                                   "uniform   mat4 u_mvp;",
                                   "uniform   float u_high;",
                                   "uniform   float u_alpha;",
                                   "",
                                   "attribute vec4 a_position;",
                                   "attribute vec2 a_texcoord;",
                                   "",
                                   "varying   vec2 v_texcoord;",
                                   "varying   float v_high;",
                                   "varying   float v_alpha;",
                                   "",
                                   "void main(void)",
                                   "{",
                                   "  v_texcoord  = a_texcoord;",
                                   "  v_high      = u_high;",
                                   "  v_alpha     = u_alpha;",
                                   "  gl_Position = u_mvp * a_position;",
                                   "}"
                                  ].join("\n");
   
   var textureFragmentShaderSource = [
                                     "#ifdef GL_ES",
                                     "precision highp float;",
                                     "#endif",
                                     "",  
                                     "uniform sampler2D s_texture;",
                                     "",
                                     "varying vec2     v_texcoord;",
                                     "varying float    v_high;",
                                     "varying float    v_alpha;",
                                     "",
                                     "void main(void)",
                                     "{",
                                     "  vec3 color = texture2D(s_texture, v_texcoord).xyz;",
                                     "  vec3 highcolor = vec3(color.x * v_high,color.y * v_high,color.z * v_high);",
                                     "  gl_FragData[0] = vec4(highcolor, v_alpha);",
                                     "}"
                                     ].join("\n");
   
  var texturetextVertexShaderSource = [
                                   "#ifdef GL_ES",
                                   "precision highp float;",
                                   "#endif",
                                   "",  
                                   "uniform   mat4 u_mvp;",
                                   "",
                                   "attribute vec4 a_position;",
                                   "attribute vec2 a_texcoord;",
                                   "",
                                   "varying   vec2 v_texcoord;",
                                   "",
                                   "void main(void)",
                                   "{",
                                   "  v_texcoord  = a_texcoord;",
                                   "  gl_Position = u_mvp * a_position;",
                                   "}"
                                  ].join("\n");
   
   var texturetextFragmentShaderSource = [
                                     "#ifdef GL_ES",
                                     "precision highp float;",
                                     "#endif",
                                     "",  
                                     "uniform sampler2D s_texture;",
                                     "",
                                     "varying vec2     v_texcoord;",
                                     "",
                                     "void main(void)",
                                     "{",
                                     "  float alpha = texture2D(s_texture,v_texcoord).a;",
                                     "  vec4 color = texture2D(s_texture, v_texcoord);",
                                     "  if(alpha < 0.3) discard;", 
                                     "  else ",
                                     "  gl_FragColor = vec4(color.rgb,1.0);",
                                     "}"
                                     ].join("\n");

//   
//   var billboardVertexShaderSource = [
//                                      "#ifdef GL_ES",
//                                      "precision highp float;",
//                                      "#endif",
//                                      "", 
//                                      "uniform   mat4 u_mvp;",
//                                      "uniform   vec3 u_color;",
//                                      "", 
//                                      "attribute vec3 a_position;",
//                                      "attribute vec2 a_texcoord;",
//                                      "",
//                                      "varying   vec3 v_color;",
//                                      "varying   vec2 v_texcoord;",
//                                      "",
//                                      "void main(void)",
//                                      "{",
//                                      "  v_texcoord = a_texcoord;",
//                                      "  v_color = u_color;",
//                                      "",
//                                      "  // Output vertex position",
//                                      "  gl_Position = u_mvp * vec4(a_position, 0.0, 1.0);",
//                                      "}"
//                                  ].join("\n");
//   
//   var billboardFragmentShaderSource = [
//                                     "#ifdef GL_ES",
//                                     "precision highp float;",
//                                     "#endif",
//                                     "",
//                                     "#extension GL_EXT_geometry_shader4 : enable",
//                                     "", 
//                                     "uniform float     u_sphere_radius;", 
//                                     "uniform vec4      u_view_light_dir;",
//                                     "", 
//                                     "varying vec3 v_vertex_light_position;", 
//                                     "varying vec4 v_eye_position;", 
//                                     "",
//                                     "void main()", 
//                                     "{", 
//                                     "    float halfsize = u_sphere_radius * 0.5;", 
//                                     "    ", 
//                                     "    gl_TexCoord[0] = gl_TexCoordIn[0][0];", 
//                                     "    gl_FrontColor = gl_FrontColorIn[0];", 
//                                     "    ", 
//                                     "    vertex_light_position = normalize(gl_LightSource[0].position.xyz);", 
//                                     "    eye_position = gl_PositionIn[0];", 
//                                     "    ", 
//                                     "    // Vertex 1", 
//                                     "    gl_TexCoord[0].st = vec2(-1.0,-1.0);", 
//                                     "    gl_Position = gl_PositionIn[0];", 
//                                     "    gl_Position.xy += vec2(-halfsize, -halfsize);", 
//                                     "    gl_Position = gl_ProjectionMatrix  * gl_Position;", 
//                                     "    EmitVertex();", 
//                                     "    ", 
//                                     "    // Vertex 2", 
//                                     "    gl_TexCoord[0].st = vec2(-1.0,1.0);", 
//                                     "    gl_Position = gl_PositionIn[0];", 
//                                     "    gl_Position.xy += vec2(-halfsize, halfsize);", 
//                                     "    gl_Position = gl_ProjectionMatrix  * gl_Position;", 
//                                     "    EmitVertex();", 
//                                     "    ", 
//                                     "    // Vertex 3", 
//                                     "    gl_TexCoord[0].st = vec2(1.0,-1.0);", 
//                                     "    gl_Position = gl_PositionIn[0];", 
//                                     "    gl_Position.xy += vec2(halfsize, -halfsize);", 
//                                     "    gl_Position = gl_ProjectionMatrix  * gl_Position;", 
//                                     "    EmitVertex();", 
//                                     "    ", 
//                                     "    // Vertex 4", 
//                                     "    gl_TexCoord[0].st = vec2(1.0,1.0);", 
//                                     "    gl_Position = gl_PositionIn[0];", 
//                                     "    gl_Position.xy += vec2(halfsize, halfsize);", 
//                                     "    gl_Position = gl_ProjectionMatrix  * gl_Position;", 
//                                     "    EmitVertex();", 
//                                     "    ", 
//                                     "    EndPrimitive();", 
//                                     }
//                                     ].join("\n");
   
   
   this.shaderPrograms[this.ShaderTypes.SIMPLE] = new SglProgram(gl, [ simpleVertexShaderSource ], [ simpleFragmentShaderSource ]);
   ros_debug(this.shaderPrograms[this.ShaderTypes.SIMPLE].log);
   this.shaderPrograms[this.ShaderTypes.SIMPLE_COLOR] = new SglProgram(gl, [ simpleColorVertexShaderSource ], [ simpleColorFragmentShaderSource ]);
   ros_debug(this.shaderPrograms[this.ShaderTypes.SIMPLE_COLOR].log);
   this.shaderPrograms[this.ShaderTypes.FLAT] = new SglProgram(gl, [ flatVertexShaderSource ], [ flatFragmentShaderSource ]);
   ros_debug(this.shaderPrograms[this.ShaderTypes.FLAT].log);
   this.shaderPrograms[this.ShaderTypes.PHONG] = new SglProgram(gl, [ phongVertexShaderSource ], [ phongFragmentShaderSource ]);  
   ros_debug(this.shaderPrograms[this.ShaderTypes.PHONG].log);
   this.shaderPrograms[this.ShaderTypes.TEXTURE] = new SglProgram(gl, [ textureVertexShaderSource ], [ textureFragmentShaderSource ]);  
   ros_debug(this.shaderPrograms[this.ShaderTypes.TEXTURE].log);
   this.shaderPrograms[this.ShaderTypes.POINT_CLOUD] = new SglProgram(gl, [ pointCloudVertexShaderSource ], [ pointCloudFragmentShaderSource ]);
   ros_debug(this.shaderPrograms[this.ShaderTypes.POINT_CLOUD].log);
   this.shaderPrograms[this.ShaderTypes.TEXTURE_TEXT] = new SglProgram(gl, [ texturetextVertexShaderSource ], [ texturetextFragmentShaderSource ]);
   ros_debug(this.shaderPrograms[this.ShaderTypes.TEXTURE_TEXT].log);
}
