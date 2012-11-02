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

ros.visualization.GLMaterial = Class.extend({
  init: function (blackTexName, whiteTexName) {
    this.emissionCol  = [ 0.0, 0.0, 0.0, 0.0 ];
    this.ambientCol   = [ 0.1, 0.1, 0.1, 1.0 ];
    this.diffuseCol   = [ 1.0, 1.0, 1.0, 1.0 ];
    this.specularCol  = [ 0.0, 0.0, 0.0, 0.0 ];
    this.shininessVal = 0.0;
   
    this.emissionMap  = blackTexName;
    this.ambientMap   = whiteTexName;
    this.diffuseMap   = whiteTexName;
    this.specularMap  = whiteTexName;
    this.shininessMap = blackTexName;

    this.name = "Collada Model";
  },
});

ros.visualization.GLNode = Class.extend({
  init: function () {
    this.meshGroupIDs = [ ];
    this.matrix       = sglIdentityM4();
    this.children     = [ ];
  },
});
   
ros.visualization.GLInstanceVisualScene = Class.extend({
  init: function () {
    this.id    = null;
    this.nodes = [ ];
  },
});

ros.visualization.GLScene = Class.extend({
  init: function () {
    this.instanceVisualSceneIDs = [ ];
  },
});

ros.visualization.GLDocument = Class.extend({
  init: function () {
    this.textures             = { };
    this.materials            = { };
    this.meshGroups           = { };
    this.instanceVisualScenes = { };
    this.scenes               = { };
  },
});
  
ros.visualization.ColladaModel = ros.visualization.Model.extend({
  init: function(gl, shader_manager, scene) 
  {
    this._super(gl, shader_manager);
    this.shader = shader_manager.ShaderTypes.TEXTURE;
    this.prog = shader_manager.shaderPrograms[this.shader];
    this.renderer = new SglMeshGLRenderer(this.prog); 

    this.gldoc = null;
    this.scene = scene;

    this.vertices = new Array();
    this.indices = new Array();

    this.is_loaded = false;
  },
    
  load: function (callback) 
  {
    var that = this;
    var daePath = this.scene.url.substr(0, this.scene.url.lastIndexOf("/"));
    this.eye = this.scene.eye;
    this.alphaValue = this.scene.alpha;
    
    //    ros_debug("loading " + daePath);
    
    function onloadDataset(domCollada) {
	//console.log(domCollada);
	var collada = getCollada(domCollada);
      if(!collada) {
        ros_error("collada document is invalid!!!");
        return;
      }
     // console.log(collada);
      //console.log(that.scene);
      
      that.gldoc = that.colladaToGLDocument(that.gl, collada, daePath, that.scene);
      if(!that.gldoc) {
        ros_error("gl document is invalid!!!");
        return;
      }
      
      // update bounding box
      that.updateBoundingBox(that.gldoc.bbox);

      that.is_loaded = true;
    }
    ros_debug(this.scene.url);
    loadXML(this.scene.url, onloadDataset); 
  },
 
  draw: function(gl, xform) 
  {

    if(this.is_loaded) {
      this.renderer.begin();
      this.drawDocument(gl, xform, this.gldoc);
      this.renderer.end();
    }
  },
 
  getBBox: function() {
    return this.getGLDocumentBBox(this.gldoc);
  },
  
  flattenIndices: function(names, sizes, values, ariety, indices) {
    var attribsCount = sizes.length;
   
    var res = {
      names   : new Array(attribsCount),
      sizes   : new Array(attribsCount),
      values  : new Array(attribsCount),
      ariety  : ariety,
      indices : [ ]
    };
   
    for (var a=0; a<attribsCount; ++a) {
      res.names[a]  = names[a];
      res.sizes[a]  = sizes[a];
      res.values[a] = [ ];
    }
   
    var indicesCount  = indices[0].length;
    var verticesCount = 0;
   
    var remap    = { };
    var idxs     = new Array(attribsCount);
   
    for (var i=0; i<indicesCount; ++i) {
      var v = indices[0][i];
      idxs[0] = v;
      var vertexString = "" + v;
      for (var a=1; a<attribsCount; ++a) {
        v = indices[a][i];
        idxs[a] = v;
        vertexString += "_" + v;
      }
      
   
      var vertexIndex = remap[vertexString];
      if (vertexIndex == undefined) {
        for (var a=0; a<attribsCount; ++a) {
          var src   = values[a];
          var dst   = res.values[a];
          var size  = res.sizes[a];
          var begin = idxs[a] * size;
          var end   = begin + size;
   
          for (var c=begin; c<end; ++c) {
            dst.push(src[c]);
          }
        }
   
        vertexIndex = verticesCount;
        remap[vertexString] = vertexIndex;
        verticesCount++;
      }
   
      res.indices.push(vertexIndex);
    }
   
    return res;
  },
   
  colladaMeshToMeshJSArray: function (colladaMesh) {
    var meshesJS = [ ];
   
    for (var t in colladaMesh.triangles) {
      var triangles = colladaMesh.triangles[t];
   
      var names   = [ ];
      var sizes   = [ ];
      var values  = [ ];
      var indices = [ ];
   
      for (var i in triangles.inputs) {
        var input  = triangles.inputs[i];
        var source = colladaMesh.sources[input.sourceID];
        var sName  = input.semantic.toLowerCase();
        if (source.set >= 0) {
          sName += source.set;
        }
        names.push(sName);
        sizes.push(source.size);
        values.push(source.buffer);
      }
   
      var ariety  = 3;
      var indices = triangles.indices;
   
      var flatMesh = this.flattenIndices(names, sizes, values, ariety, indices);
   
      var meshJS = new SglMeshJS();
   
      var attribsCount = flatMesh.names.length;
      for (var a=0; a<attribsCount; ++a) {
        meshJS.addVertexAttribute(flatMesh.names[a], flatMesh.sizes[a], flatMesh.values[a]);

        if(flatMesh.names[a] == "position") {
          this.vertices.push(flatMesh.values[a]);
          this.indices.push(flatMesh.indices);
        }
      }
      var primName = triangles.materialSym || "triangles";
      meshJS.addIndexedPrimitives(primName, SGL_TRIANGLES_LIST, flatMesh.indices);
   
      meshesJS.push(meshJS);
    }
   
    return meshesJS;
  },
 
  colladaNodeToGL: function (node) {
    if (!node) return null;
   
    var gn = new ros.visualization.GLNode();
   
    gn.meshGroupIDs = node.instanceGeometries;
    gn.matrix = node.matrix;
    for (var i=0, n=node.children.length; i<n; ++i) {
      var child = this.colladaNodeToGL(node.children[i]);
      if (!child) continue;
      gn.children.push(child);
    }
   
    return gn;
  },
   
  getGLMeshBBox: function (mesh, matrix, doc) {
    var bbox = mesh.bbox.transformed(matrix);
    return bbox;
  },
   
  getGLMeshArrayBBox: function (meshArray, matrix, doc) {
    var bbox = new SglBox3();
    for (var m in meshArray) {
      var mesh = meshArray[m];
      bbox.addBox(this.getGLMeshBBox(mesh, matrix, doc));
    }
    return bbox;  
  },
   
  getGLNodeBBox: function (node, matrix, doc) {
    var mat = sglMulM4(matrix, node.matrix);
    var bbox = new SglBox3();
    for (var c in node.children) {
      var child = node.children[c];
      bbox.addBox(this.getGLNodeBBox(child, mat, doc));
    }
    for (var m in node.meshGroupIDs) {
      var meshArray = doc.meshGroups[node.meshGroupIDs[m].geometryID];
      bbox.addBox(this.getGLMeshArrayBBox(meshArray, mat, doc));
    }
    return bbox;  
  },
   
  getGLVisualSceneBBox: function (visualScene, matrix, doc) {
    var bbox = new SglBox3();
   // console.log(visualScene);
    for (var n in visualScene.nodes) {
      var node = visualScene.nodes[n];
      if(node) {
    	  bbox.addBox(this.getGLNodeBBox(node, matrix, doc));
      }
    }
    return bbox;  
  },
   
  getGLSceneBBox: function (scene, matrix, doc) {
    var bbox = new SglBox3();
    for (var v in scene.instanceVisualSceneIDs) {
      var visualScene = doc.instanceVisualScenes[scene.instanceVisualSceneIDs[v]];
      if(visualScene) {
    	  bbox.addBox(this.getGLVisualSceneBBox(visualScene, matrix, doc));
      }
    }
    return bbox;  
  },
   
  getGLDocumentBBox: function (doc) {
    var matrix = sglIdentityM4();
    var bbox   = new SglBox3();
    for (var s in doc.scenes) {
      var scene = doc.scenes[s];
      bbox.addBox(this.getGLSceneBBox(scene, matrix, doc));
    }
    return bbox;
  },
   
  getGLMaterial: function (gl, blackTexName, whiteTexName, material) {
    var res = new ros.visualization.GLMaterial(blackTexName, whiteTexName);
   
    if (material.emissionCol) {
      res.emissionCol = material.emissionCol;
    }
    if (material.emissionMap) {
      res.emissionCol = [ 0.0, 0.0, 0.0, 0.0 ];
      res.emissionMap = material.emissionMap;
    }
   
    if (material.ambientCol) {
      res.ambientCol = material.ambientCol;
    }
    if (material.ambientMap) {
      //res.ambientCol = [ 0.0, 0.0, 0.0, 0.0 ];
      res.ambientMap = material.ambientMap;
    }
   
    if (material.diffuseCol) {
      res.diffuseCol = material.diffuseCol;
    }
    if (material.diffuseMap) {
      //res.diffuseCol = [ 1.0, 1.0, 1.0, 1.0 ];
      res.diffuseMap = material.diffuseMap;
    }
   
    if (material.specularCol) {
      res.specularCol = material.specularCol;
    }
    if (material.specularMap) {
      res.specularCol = [ 1.0, 1.0, 1.0, 1.0 ];
      res.specularMap = material.specularMap;
    }
   
    if (material.shininessVal) {
      res.shininessVal = material.shininessVal;
    }
    if (material.shininessMap) {
      res.shininessVal = [ 0.0 ];
      res.shininessMap = material.shininessMap;
    }
   
    return res;
  },
   
  colladaToGLDocument: function (gl, collada, colladaBasePath, daeSource) {
    if (!gl || !collada) return null;
   
    var gldoc = new ros.visualization.GLDocument();
   
    var texOpts = {
      minFilter : gl.LINEAR_MIPMAP_LINEAR,
      magFilter : gl.LINEAR,
      wrapS     : gl.REPEAT,
      wrapT     : gl.REPEAT,
      generateMipmap : true
    };
  
  //  var texOpts = {
  //      minFilter : gl.LINEAR,
  //      magFilter : gl.LINEAR,
  //      wrapS     : gl.REPEAT,
  //      wrapT     : gl.REPEAT,
  //      generateMipmap : false
  //    };
    
    for (var t in collada.textures) {
      var texFile = colladaBasePath + "/" + collada.textures[t];
      
      
      // convert tif's to png's
      if (texFile.substr(-4).toLowerCase() == ".tif")
        texFile = texFile.substr(0, texFile.length-3) + "png";
      
      gldoc.textures[t] = new SglTexture2D(gl, texFile, texOpts);
    }
   
    var dummyBlackTexels = new Uint8Array([
      0, 0, 0, 0
    ]);
    var dummyBlackTex = new SglTexture2D(gl, gl.RGBA, 1, 1, gl.RGBA, gl.UNSIGNED_BYTE, dummyBlackTexels, texOpts);
    var dummyBlackTexName = "dummyBlack";
    while (gldoc.textures[dummyBlackTexName]) {
      dummyBlackTexName += "_";
    }
    gldoc.textures[dummyBlackTexName] = dummyBlackTex;
    gldoc.blackTex = dummyBlackTex;
   
    var dummyWhiteTexels = new Uint8Array([
      255, 255, 255, 255
    ]);
    var dummyWhiteTex = new SglTexture2D(gl, gl.RGBA, 1, 1, gl.RGBA, gl.UNSIGNED_BYTE, dummyWhiteTexels, texOpts);
    var dummyWhiteTexName = "dummyWhite";
    while (gldoc.textures[dummyWhiteTexName]) {
      dummyWhiteTexName += "_";
    }
    gldoc.textures[dummyWhiteTexName] = dummyWhiteTex;
    gldoc.whiteTex = dummyWhiteTex;
   
    for (var m in collada.materials) {
      var material = collada.materials[m];
      var glMat = this.getGLMaterial(gl, dummyBlackTexName, dummyWhiteTexName, material);
      if (daeSource.specular) {
        glMat.specularCol = daeSource.specular.slice();
      }
      gldoc.materials[m] = glMat;

		// check if a texture needs to be created as well to color the mesh
                if(!gldoc.textures[m]) {
		    curTexels = new Uint8Array([
      		(material.ambientCol[0] * 255), material.ambientCol[1] * 255, material.ambientCol[2]* 255, material.ambientCol[3]* 255
    		]);
		    gldoc.textures[m] = new SglTexture2D(gl, gl.RGBA, 1, 1, gl.RGBA,  gl.UNSIGNED_BYTE, curTexels, texOpts);
		}
    }
   
    for (var g in collada.geometries) {
      var geometry = collada.geometries[g];
      var meshesGL = [ ];
      for (var m in geometry.meshes) {
        var mesh = geometry.meshes[m];
        var meshesJS = this.colladaMeshToMeshJSArray(mesh);
        for (var mj in meshesJS) {
          var meshJS = meshesJS[mj];
          var meshGL = sglMeshJStoGL(gl, meshJS);
          meshGL.bbox = sglMeshJSCalculateBBox(meshJS);
          meshesGL.push(meshGL);
        }
      }
      gldoc.meshGroups[g] = meshesGL;
    }
   
    for (var v in collada.instanceVisualScenes) {
      var instanceVisualScenes = collada.instanceVisualScenes[v];
      var gv = new ros.visualization.GLInstanceVisualScene();
      gv.id = instanceVisualScenes.id;
      for (var i=0, n=instanceVisualScenes.nodes.length; i<n; ++i) {
        var node = instanceVisualScenes.nodes[i];
        gv.nodes.push(this.colladaNodeToGL(node));
      }
      gldoc.instanceVisualScenes[v] = gv;
    }
   
    for (var s in collada.scenes) {
      var scene = collada.scenes[s];
      var gs = new ros.visualization.GLScene();
      gs.instanceVisualSceneIDs = scene.instanceVisualSceneIDs.slice();
      gldoc.scenes[s] = gs;
    }
    //console.log(gldoc);
    gldoc.bbox = this.getGLDocumentBBox(gldoc);

    
    gldoc.sceneInfo = daeSource;
   
    return gldoc;
  },

  getTexture: function(doc, texName, defaultTex) {
    var tex = doc.textures[texName];
    if (!tex || !tex.isValid) {
      tex = defaultTex;
    }
    return tex;
  },

  drawMesh: function(gl, xform, material, mesh, primitive, doc) {
        
    if(this.shader == this.shader_manager.ShaderTypes.TEXTURE) {
      var uniforms = {
          u_mvp : xform.modelViewProjectionMatrix,
          u_high : this.highlightPass, 
          u_alpha : this.alphaValue
        };
  
        var samplers = {

          s_texture : this.getTexture(doc, material.diffuseMap, doc.whiteTex),
        };
  
        //sglRenderMeshGLPrimitives(mesh, "triangles", this.prog, null, uniforms, samplers);
  
        this.renderer.setUniforms(uniforms);
        this.renderer.setSamplers(samplers);
    }
    else {
      var uniforms = {
          u_model_view_projection_mat : xform.modelViewProjectionMatrix,
          u_model_view_mat : xform.modelViewMatrix,
          u_view_normal_mat : xform.viewSpaceNormalMatrix,
          u_view_light_dir : [ 0.0, 0.0, -1.0, ((doc.sceneInfo.light) ? (0.0) : (1.0)) ],
          u_emission : material.emissionCol,
          u_ambient : material.ambientCol,
          u_diffuse : material.diffuseCol,
          u_specular : material.specularCol,
          u_shininess : material.shininessVal
        };
  
        var samplers = {
          s_emission : this.getTexture(doc, material.emissionMap, doc.blackTex),
          s_ambient : this.getTexture(doc, material.ambientMap, doc.whiteTex),
          s_diffuse : this.getTexture(doc, material.diffuseMap, doc.whiteTex),
          s_specular : this.getTexture(doc, material.specularMap, doc.whiteTex),
          s_shininess : this.getTexture(doc, material.shininessMap, doc.blackTex)
        };
  
        //sglRenderMeshGLPrimitives(mesh, "triangles", this.prog, null, uniforms, samplers);
  
        this.renderer.setUniforms(uniforms);
        this.renderer.setSamplers(samplers);
    }
    
    this.renderer.beginMesh(mesh);
    for ( var p in mesh.connectivity.primitives) {
      if (p == primitive) {
        this.renderer.beginPrimitives(p);
        this.renderer.render();
        this.renderer.endPrimitives();
      }
    }
    this.renderer.endMesh();
  },

  drawMeshArray: function(gl, xform, material, meshArray, primitive, doc) {
    for ( var m in meshArray) {
      var mesh = meshArray[m];
      this.drawMesh(gl, xform, material, mesh, primitive, doc);
    }
  },

  drawNode: function(gl, xform, node, doc) {
    xform.model.push();
    xform.model.multiply(node.matrix);
    for ( var c in node.children) {
      var child = node.children[c];
      this.drawNode(gl, xform, child, doc);
    }
    for ( var m in node.meshGroupIDs) {
      var meshGroup = node.meshGroupIDs[m];
      var material = doc.materials[meshGroup.materialID];
      var meshArray = doc.meshGroups[meshGroup.geometryID];
      var primitive = meshGroup.primitiveID;
   //   console.log(meshGroup);
    //  console.log(doc);
      this.drawMeshArray(gl, xform, material, meshArray, primitive, doc);
    }
    xform.model.pop();
  },

  drawVisualScene: function(gl, xform, visualScene, doc) {
    xform.model.push();
    for ( var n in visualScene.nodes) {
      var node = visualScene.nodes[n];
      this.drawNode(gl, xform, node, doc);
    }
    xform.model.pop();
  },

  drawScene: function(gl, xform, scene, doc) {
    xform.model.push();
    for ( var v in scene.instanceVisualSceneIDs) {
      var visualScene = doc.instanceVisualScenes[scene.instanceVisualSceneIDs[v]];
      if(visualScene) {
        this.drawVisualScene(gl, xform, visualScene, doc);
      }
    }
    xform.model.pop();
  },

  drawDocument: function(gl, xform, doc) {
      xform.model.push();
      for ( var s in doc.scenes) {
        var scene = doc.scenes[s];
        this.drawScene(gl, xform, scene, doc);
      }
      xform.model.pop();
  },

  intersectRay: function(start, end) {
    var r;
    for(var v in this.vertices)
    {
      var vertexList = this.vertices[v];
      var indexList = this.indices[v];

      r = this.intersectRayOneMesh(start, end,vertexList, indexList);

      if(r.intersected)
        return r;
    }

    return r;
  },
});

