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

function loadXML(url, callback) {
	var async = (callback) ? (true) : (false);
	var req = new XMLHttpRequest();

	req.overrideMimeType("text/xml");

	if (async) {
		req.onreadystatechange = function() {
			if (req.readyState == 4) {
			  callback(req.responseXML);
			}
		};
	}

	req.open("GET", url, async);
	req.send(null);

	var ret = null;
	if (!async) {
		ret = req.responseXML;
	}

	return ret;
}

function getXmlElementById(xml, id) {
	if (!id) return null;

	var tags = xml.getElementsByTagName("*");
	var n = tags.length;

	for(var i=0; i<n; ++i) {
		if (tags[i].getAttribute("id") == id) {
			return tags[i];
		}
	}

	return null;
}

function getXmlElementBySId(xml, id) {
	if (!id) return null;

	var tags = xml.getElementsByTagName("*");
	var n = tags.length;

	for(var i=0; i<n; ++i) {
		if (tags[i].getAttribute("sid") == id) {
			return tags[i];
		}
	}

	return null;
}

function ColladaSource() {
	this.id     = null;
	this.size   = 0;
	this.stride = 0;
	this.offset = 0;
	this.buffer = [ ];
}

function ColladaInput() {
	this.semantic = null
	this.sourceID = null;
	this.offset   = -1;
	this.set      = -1;
}

function ColladaVertices() {
	this.id      = null;
	this.inputs  = [ ];
}

function ColladaTriangles() {
	this.inputs  = [ ];
	this.indices = [ ];
	this.materialSym = null;
}

function ColladaMesh() {
	this.sources   = { };
	this.triangles = [ ];
}

function ColladaGeometry() {
	this.id     = null;
	this.meshes = [ ];
}

function ColladaMaterial() {
	this.emissionCol  = [ 0.0, 0.0, 0.0, 0.0 ];
	this.ambientCol   = [ 0.1, 0.1, 0.1, 1.0 ];
	this.diffuseCol   = [ 1.0, 1.0, 1.0, 1.0 ];
	this.specularCol  = [ 0.0, 0.0, 0.0, 0.0 ];
	this.shininessVal = 0.0;

	this.emissionMap  = null;
	this.ambientMap   = null;
	this.diffuseMap   = null;
	this.specularMap  = null;
	this.shininessMap = null;
}

function ColladaInstanceGeometry() {
	this.materialID  = null;
	this.geometryID  = null;
	this.primitiveID = null;
}

function ColladaNode() {
	this.instanceGeometries = [ ];
	this.matrix             = sglIdentityM4();
	this.children           = [ ];
}

function ColladaInstanceVisualScene() {
	this.id    = null;
	this.nodes = [ ];
}

function ColladaScene() {
	this.instanceVisualSceneIDs = [ ];
}

function ColladaDocument() {
  this.unitScale            = 1.0;
	this.geometries           = { };
	this.textures             = { };
	this.materials            = { };
	this.instanceVisualScenes = { };
	this.scenes               = { };
}

function getColladaArray(domRoot, domArray, collada, parseFunc) {
	if (!domRoot || !domArray || !collada) return null;

	if (!parseFunc) {
		parseFunc = function(value) { return value };
	}

	var domElem = domArray.firstChild;
	var prev = "";
	var output = [ ];
	var currentArray = null;

	while (domElem) {
		currentArray = (prev + domElem.nodeValue).replace(/\s+/g," ").replace(/^\s+/g,"").split(" ");
		domElem = domElem.nextSibling;

		if (currentArray[0] == "") {
			currentArray.unshift();
		}

		if (domElem) {
			prev = currentArray.pop();
		}

		for (var i=0, n=currentArray.length; i<n;i++) {
			output.push(parseFunc(currentArray[i]));
		}
	}

	return output;
}

function getXmlFirstTag(domNode, tagName) {
	var elems = domNode.getElementsByTagName(tagName);
	if (!elems) return null;
	if (elems.length <= 0) return null
	return elems[0];
}

function getColladaIntArray(domRoot, domArray, collada) {
	return getColladaArray(domRoot, domArray, collada, parseInt);
}

function getColladaFloatArray(domRoot, domArray, collada) {
	return getColladaArray(domRoot, domArray, collada, parseFloat);
}

function getColladaTexture(domRoot, domImage, collada) {
	if (!domRoot || !domImage || !collada) return null;

	var domInitFrom = getXmlFirstTag(domImage, "init_from");
	if (!domInitFrom) return null;

	var domTexFileName = domInitFrom.firstChild;
	if (!domTexFileName) return null;

	var texture = domTexFileName.data;
	if (!texture || (texture.length <= 0)) return null;

	return texture;
}

function getColladaTextures(domRoot, collada) {
	if (!domRoot || !collada) return null;

	for (var textureURL in collada.textures) {
		var texture = getColladaTexture(domRoot, getXmlElementById(domRoot, textureURL), collada);
		if (texture) {
			collada.textures[textureURL] = texture;
		}
		else {
			delete collada.textures[textureURL];
		}
	}
}

function getColladaTexturesID(domRoot, domTexture, collada) {
	if (!domRoot || !domTexture || !collada) return null;
}

function getColladaMaterialParam(domRoot, domEffect, domMaterialParam, collada) {
	var res = {
		textureID : null,
		value     : null
	};

	if (!domRoot || !domMaterialParam) return res;

	var domFloat = getXmlFirstTag(domMaterialParam, "float");
	if (domFloat) {
		res.value = getColladaFloatArray(domRoot, domFloat, collada);
		if (res.value) {
			if (res.value.length > 0) {
				res.value = [ res.value[0] ];
			}
		}
	}

	var domColor = getXmlFirstTag(domMaterialParam, "color");
	if (domColor) {
		res.value = getColladaFloatArray(domRoot, domColor, collada);
		if (res.value) {
			if (res.value.length == 3) {
				res.value.push(1.0);
			}
		}
	}

	var domTexture = getXmlFirstTag(domMaterialParam, "texture");
	if (domTexture) {
		var samplerSID = domTexture.getAttribute("texture");
		if (samplerSID) {
			var domSampler = getXmlElementBySId(domEffect, samplerSID);
			if (domSampler) {
				var domSource = getXmlFirstTag(domSampler, "source");
				if (domSource) {
					var surfaceSID = domSource.firstChild;
					if (surfaceSID) {
						var ssid = surfaceSID.data;
						var domSurface = getXmlElementBySId(domEffect, ssid);
						if (domSurface) {
							var domInitFrom = getXmlFirstTag(domSurface, "init_from");
							if (domInitFrom) {
								var textureID = domInitFrom.firstChild;
								if (textureID) {
									var tid = textureID.data;
									if (tid.length > 0) {
										res.textureID = tid;
										collada.textures[tid] = null;
									}
								}
							}
						}
					}
				}
			}
			else {
				if (samplerSID.length > 0) {
					res.textureID = samplerSID;
					collada.textures[samplerSID] = null;
				}
			}
		}
	}

	return res;
}

function getColladaMaterialTypeString(domCommonTechnique) {
	if (!domCommonTechnique) return null;

	var matType = null;
	var domElem = domCommonTechnique.firstChild;

	while (domElem && (!matType)) {
		switch (domElem.tagName) {
			case "lambert":
			case "blinn":
			case "phong":
				matType = domElem.tagName;
			break;

			default:
			break;
		}

		domElem = domElem.nextSibling;
	}

	return matType;
}

function getColladaMaterial(domRoot, domMaterial, collada) {
	if (!domRoot || !domMaterial || !collada) return null;

	var material = new ColladaMaterial();

	var domInstanceEffect = getXmlFirstTag(domMaterial, "instance_effect");
	if (!domInstanceEffect){
		ros_debug("In getColladaMaterial, domInstanceEffect is null");
		return null;
	}

	var effectURL = domInstanceEffect.getAttribute("url");
	if (!effectURL) {
		ros_debug("In getColladaMaterial, effectURL is null");
		return null;
	}

	effectURL = effectURL.substr(1);
	if (effectURL.length <= 0) {
		ros_debug("In getColladaMaterial, effectURL length is <=0 ");
		return null;
	}

	var domEffect = getXmlElementById(domRoot, effectURL);
	if (!domEffect) {
		ros_debug("In getColladaMaterial, domEffect is null");
		return null;
	}

	var domTechniques = domEffect.getElementsByTagName("technique");
	if (!domTechniques) {
		ros_debug("In getColladaMaterial, domTechniques is null");
		return null;
	}

	var domCommonTechnique = null;
	//console.log(domTechniques.length);
	for (var i=0, n=domTechniques.length; i<n; ++i) {
		//console.log(domTechniques[i].getAttribute("sid"))
		if (domTechniques[i].getAttribute("sid") == "common" || 
		    domTechniques[i].getAttribute("sid") == "COMMON") {
			domCommonTechnique = domTechniques[i]
			break;
		}
	}
	if (!domCommonTechnique) {
		ros_debug("In getColladaMaterial, domCommonTechnique is null");
		return null;
	}

	var domEmission   = getXmlFirstTag(domCommonTechnique, "emission");
	var emissionParam = getColladaMaterialParam(domRoot, domEffect, domEmission, collada);
	// check if a valid ID was found
	if(!emissionParam.textureID)
		emissionParam.textureID = domMaterial.getAttribute("id");
	material.emissionCol = emissionParam.value;
	material.emissionMap = emissionParam.textureID;

	var domAmbient   = getXmlFirstTag(domCommonTechnique, "ambient");
	var ambientParam = getColladaMaterialParam(domRoot, domEffect, domAmbient, collada);
	// check if a valid ID was found
	if(!ambientParam.textureID)
		ambientParam.textureID = domMaterial.getAttribute("id");
	material.ambientCol = ambientParam.value;
	material.ambientMap = ambientParam.textureID;

	var domDiffuse   = getXmlFirstTag(domCommonTechnique, "diffuse");
	var diffuseParam = getColladaMaterialParam(domRoot, domEffect, domDiffuse, collada);
	// check if a valid ID was found
	if(!diffuseParam.textureID)
		diffuseParam.textureID = domMaterial.getAttribute("id");
	material.diffuseCol = diffuseParam.value;
	material.diffuseMap = diffuseParam.textureID;

	var domSpecular   = getXmlFirstTag(domCommonTechnique, "specular");
	var specularParam = getColladaMaterialParam(domRoot, domEffect, domSpecular, collada);
	// check if a valid ID was found
	if(!specularParam.textureID)
		specularParam.textureID = domMaterial.getAttribute("id");
	material.specularCol = specularParam.value;
	material.specularMap = specularParam.textureID;

	var domShininess   = getXmlFirstTag(domCommonTechnique, "shininess");
	var shininessParam = getColladaMaterialParam(domRoot, domEffect, domShininess, collada);
	material.shininessVal = shininessParam.value;
	if (shininessParam.value) {
		if (shininessParam.value.length > 0) {
			material.shininessVal = shininessParam.value[0];
		}
	}
	// check if a valid ID was found
	if(!shininessParam.textureID)
		shininessParam.textureID = domMaterial.getAttribute("id");
	material.shininessMap = shininessParam.textureID;

	var commonMaterialType = getColladaMaterialTypeString(domCommonTechnique);
	switch (commonMaterialType) {
		case "lambert":
			material.specularCol  = [ 0.0, 0.0, 0.0, 0.0 ];
			material.specularMap  = null;
			material.shininessVal = 0.0;
			material.shininessMap = null;
		break;

		case "blinn":
			material.shininessVal *= 128.0;
		break;

		case "phong":
		break;

		default:
		break;
	}

	return material;
}

function getColladaMaterials(domRoot, collada) {
	if (!domRoot || !collada) return null;

	for (var materialURL in collada.materials) {
		//console.log(materialURL);
		//console.log(getXmlElementById(domRoot, materialURL));
		var material = getColladaMaterial(domRoot, getXmlElementById(domRoot, materialURL), collada);
	//	console.log(material);
		if (material) {
			collada.materials[materialURL] = material;
		}
		else {
			delete collada.materials[materialURL];
		}
	}
}

function getColladaSource(domRoot, domSource, collada) {
	if (!domRoot || !domSource || !collada) return null;

	var source = new ColladaSource();
	source.id = domSource.getAttribute("id");
	var domTrechniqueCommon = domSource.getElementsByTagName("technique_common");
	if (!domTrechniqueCommon) return null;

	var domAccessors = domSource.getElementsByTagName("accessor");
	if (!domAccessors) return null;
	if (domAccessors.length <= 0) return null;

	var domAccessor = domAccessors[0];
	if (!domAccessor) return null;

	source.offset = domAccessor.getAttribute("offset");
	source.stride = domAccessor.getAttribute("stride");

	var domParams = domAccessor.getElementsByTagName("param");
	if (!domParams) return null;

	source.size   = domParams.length;

	var sourceArrayURL = domAccessor.getAttribute("source");
	if (!sourceArrayURL) return null;
	sourceArrayURL = sourceArrayURL.substr(1);
	if (sourceArrayURL.length <= 0) return null;

	var domArray = getXmlElementById(domSource, sourceArrayURL);
	if (!domArray) return null;

	source.buffer = getColladaFloatArray(domRoot, domArray, collada);

	return source;
}

function getColladaInput(domRoot, domInput, collada) {
	if (!domRoot || !domInput || !collada) return null;

	var input  = new ColladaInput();

	input.semantic = domInput.getAttribute("semantic");

	var str = domInput.getAttribute("source");
	if (str) {
		str = str.substr(1);
	}
	input.sourceID = ((str) ? (str) : (null));

	str = domInput.getAttribute("offset");
	input.offset = ((str) ? (parseInt(str)) : (-1));

	str = domInput.getAttribute("set");
	input.set = ((str) ? (parseInt(str)) : (-1));

	return input;
}

function getColladaVertices(domRoot, domVertices, collada) {
	if (!domRoot || !domVertices || !collada) return null;

	var vertices = new ColladaVertices();
	vertices.id = domVertices.getAttribute("id");

	var domElem = domVertices.firstChild;
	var input   = null;

	while (domElem) {
		switch (domElem.tagName) {
			case "input" :
				input = getColladaInput(domRoot, domElem, collada);
				if (input) {
					vertices.inputs.push(input);
				}
			break;

			default :
			break;
		}

		domElem = domElem.nextSibling;
	}

	if (vertices.inputs.length <= 0) {
		return null;
	}

	return vertices;
}

function getColladaTriangles(domRoot, domTriangles, collada) {
	if (!domRoot || !domTriangles || !collada) return null;

	var triangles = new ColladaTriangles();

	triangles.materialSym = domTriangles.getAttribute("material");

	var domElem = domTriangles.firstChild;
	var input   = null;
	var indices = null;

	while (domElem) {
		switch (domElem.tagName) {
			case "input" :
				input = getColladaInput(domRoot, domElem, collada);
				if (input) {
					triangles.inputs.push(input);
				}
			break;

			case "p" :
				indices = getColladaIntArray(domRoot, domElem, collada);
				if (indices) {
					if (indices.length > 0) {
						triangles.indices = indices;
					}
				}
			break;

			default :
			break;
		}

		domElem = domElem.nextSibling;
	}

	if ((triangles.inputs.length <= 0) || (triangles.indices.length <= 0)) {
		return null;
	}

	return triangles;
}

function getColladaPolylistAsTriangles(domRoot, domPolylist, collada) {
	if (!domRoot || !domPolylist || !collada) return null;

	var triangles = new ColladaTriangles();

	triangles.materialSym = domPolylist.getAttribute("material");

	var domElem = domPolylist.firstChild;
	var input   = null;
	var vcount  = null;
	var indices = null;

	while (domElem) {
		switch (domElem.tagName) {
			case "input" :
				input = getColladaInput(domRoot, domElem, collada);
				if (input) {
					triangles.inputs.push(input);
				}
			break;

			case "vcount" :
				vcount = getColladaIntArray(domRoot, domElem, collada);
			break;

			case "p" :
				indices = getColladaIntArray(domRoot, domElem, collada);
			break;

			default :
			break;
		}

		domElem = domElem.nextSibling;
	}

	var inputsCount = triangles.inputs.length;
	if ((inputsCount <= 0) || !vcount || (vcount.length <=0) || !indices || (indices.length <= 0)) {
		return null;
	}

	var maxInputOffset = -1;
	for (var j=0; j<inputsCount; ++j) {
		if (maxInputOffset < triangles.inputs[j].offset) {
			maxInputOffset = triangles.inputs[j].offset;
		}
	}
	if (maxInputOffset < 0) return null;

	var indexStride = maxInputOffset + 1;

	function readIndex(src, offset, cnt, dst) {
		for (var i=0; i<cnt; ++i) {
			dst[i] = src[offset + i];
		}
	}

	function appendIndex(dst, cnt, src) {
		for (var i=0; i<cnt; ++i) {
			dst.push(src[i]);
		}
	}

	var triIndices = [ ];
	var k = 0;
	var vc = 0;
	var i0 = new Array(indexStride);
	var i1 = new Array(indexStride);
	var i2 = new Array(indexStride);
	var iTmp = null;

	for (var v=0, vn=vcount.length; v<vn; ++v) {
		vc = vcount[v];
		if (vc < 3) continue;

		readIndex(indices, k, indexStride, i0); k += indexStride;
		readIndex(indices, k, indexStride, i1); k += indexStride;
		for (i=2; i<vc; ++i) {
			readIndex(indices, k, indexStride, i2); k += indexStride;
			appendIndex(triIndices, indexStride, i0);
			appendIndex(triIndices, indexStride, i1);
			appendIndex(triIndices, indexStride, i2);
			iTmp = i1;
			i1 = i2;
			i2 = iTmp;
		}
	}

	triangles.indices = triIndices;

	return triangles;
}

function getColladaMesh(domRoot, domMesh, collada) {
	if (!domRoot || !domMesh || !collada) return null;

	var mesh = new ColladaMesh();

	var domElem   = domMesh.firstChild;
	var source    = null;
	var sourcesCount = 0;
	var vertices  = null;
	var triangles = null;

	while (domElem) {
		switch (domElem.tagName) {
			case "source" :
				source = getColladaSource(domRoot, domElem, collada);
				if (source) {
					mesh.sources[source.id] = source;
					sourcesCount++;
				}
			break;

			case "vertices" :
				vertices = getColladaVertices(domRoot, domElem, collada);
			break;

			case "triangles" :
				triangles = getColladaTriangles(domRoot, domElem, collada);
				if (triangles) {
					mesh.triangles.push(triangles);
				}
			break;

			case "polylist" :
				triangles = getColladaPolylistAsTriangles(domRoot, domElem, collada);
				if (triangles) {
					mesh.triangles.push(triangles);
				}
			break;

			default :
			break;
		}

		domElem = domElem.nextSibling;
	}

	var sortInputsByOffset = function(a, b) {
		return (a.offset - b.offset);
	}

	var vertInputCount = (vertices) ? (vertices.inputs.length) : (0);
	var numTriSets     = mesh.triangles.length;
	for (var i=0; i<numTriSets; ++i) {
		var triangles = mesh.triangles[i];
		var triInputsCount = triangles.inputs.length;

		var maxInputOffset = -1;
		for (var j=0; j<triInputsCount; ++j) {
			if (maxInputOffset < triangles.inputs[j].offset) {
				maxInputOffset = triangles.inputs[j].offset;
			}
		}
		if (maxInputOffset < 0) continue;
		var indexStride = maxInputOffset + 1;

		var newTriInputs = [ ];
		for (var j=0; j<triInputsCount; ++j) {
			var input = triangles.inputs[j];
			if ((vertInputCount > 0) && (input.sourceID == vertices.id)) {
				for (var k=0; k<vertInputCount; ++k) {
					var vertInput = vertices.inputs[k];
					vertInput.offset = input.offset;
					newTriInputs.push(vertInput);
				}
			}
			else {
				newTriInputs.push(input);
			}
		}

		newTriInputs.sort(sortInputsByOffset);

		var triInputsNewCount = newTriInputs.length;
		var newIndices = new Array(triInputsNewCount);
		for (var j=0; j<triInputsNewCount; ++j) {
			newIndices[j] = [ ];
		}

		var triIndicesCount = triangles.indices.length;
		for (var j=0; j<triInputsNewCount; ++j) {
			var input = newTriInputs[j];
			for (var k=input.offset; k<triIndicesCount; k+=indexStride) {
				newIndices[j].push(triangles.indices[k]);
			}
		}

		triangles.inputs  = newTriInputs;
		triangles.indices = newIndices;
	}

	return mesh;
}

function getColladaGeometry(domRoot, domGeometry, collada) {
	if (!domRoot || !domGeometry || !collada) return null;

	var geometry = new ColladaGeometry();
	geometry.id = domGeometry.getAttribute("id");
	if (!geometry.id) return null;

	var domElem = domGeometry.firstChild;
	var mesh = null;

	while (domElem) {
		switch (domElem.tagName) {
			case "mesh" :
				mesh = getColladaMesh(domRoot, domElem, collada);
				if (mesh) {
					geometry.meshes.push(mesh);
				}
			break;

			default :
			break;
		}

		domElem = domElem.nextSibling;
	}

	if (geometry.meshes.length <= 0) {
		return null;
	}

	return geometry;
}

function getColladaGeometries(domRoot, collada) {
	if (!domRoot || !collada) return null;

	for (var geometryURL in collada.geometries) {
		var geometry = getColladaGeometry(domRoot, getXmlElementById(domRoot, geometryURL), collada);
		if (geometry) {
			collada.geometries[geometryURL] = geometry;
		}
		else {
			delete collada.geometries[geometryURL];
		}
	}
}

function getColladaInstanceGeometry(domRoot, domInstanceGeometry, collada) {
	if (!domRoot || !domInstanceGeometry || !collada) return null;

	var geometryURL = domInstanceGeometry.getAttribute("url");
	if (!geometryURL) return null;

	geometryURL = geometryURL.substr(1);
	if (geometryURL.length <= 0) return null;

	var instanceGeometries = [ ];

	collada.geometries[geometryURL] = null;

	var domInstanceMaterials = domInstanceGeometry.getElementsByTagName("instance_material");
	if(domInstanceMaterials.length==0)
	{
	  ros_error("no instance_material defined for " + geometryURL + " geometry.");
	}
	
	for (var i=0, n=domInstanceMaterials.length; i<n; ++i) {
		var domInstanceMaterial = domInstanceMaterials[i];
		if (!domInstanceMaterial) continue;

		var instanceGeometry = new ColladaInstanceGeometry();
		instanceGeometry.geometryID = geometryURL;

		var materialURL = domInstanceMaterial.getAttribute("target");
		if (materialURL) {
			materialURL = materialURL.substr(1);
			if (materialURL.length > 0) {
				instanceGeometry.materialID = materialURL;
				collada.materials[materialURL] = null;
			}
		}
		var materialSym = domInstanceMaterial.getAttribute("symbol");
		if (materialSym.length > 0) {
			instanceGeometry.primitiveID = materialSym;
		}

		instanceGeometries.push(instanceGeometry);
	}

	return instanceGeometries;
}

function getColladaNode(domRoot, domNode, collada) {
	if (!domRoot || !domNode || !collada) return null;

	var domElem  = domNode.firstChild;

	var node      = new ColladaNode();
	var matrix    = sglIdentityM4();
	var childNode = null;
	var instanceGeometries = null;
	var arr = null;

	// apply unit scale to all geometry
	matrix = sglMulM4(matrix, sglScalingM4V([collada.unitScale,collada.unitScale,collada.unitScale]));
	
	while (domElem) {
		var nextElem = domElem.nextSibling;
		//ros_error(domElem.tagName);
		switch (domElem.tagName) {
			case "node" :
				childNode = getColladaNode(domRoot, domElem, collada);
				if (childNode) {
					node.children.push(childNode);
				}
			break;

	    case "matrix" :
	      arr = getColladaFloatArray(domRoot, domElem, collada);
	      if (arr) {
	        if (arr.length == 16) {
	          m = sglM4(arr);
	          matrix = sglMulM4(matrix, sglTransposeM4(m));
	        }
	      }
	    break;
	      
			case "translate" :
				arr = getColladaFloatArray(domRoot, domElem, collada);
				if (arr) {
					if (arr.length == 3) {
						matrix = sglMulM4(matrix, sglTranslationM4V(arr));
					}
				}
			break;

			case "rotate" :
				arr = getColladaFloatArray(domRoot, domElem, collada);
				if (arr) {
					if (arr.length == 4) {
						matrix = sglMulM4(matrix, sglRotationAngleAxisM4V(sglDegToRad(arr[3]), arr.slice(0, 3)));
					}
				}
			break;

			case "scale" :
				arr = getColladaFloatArray(domRoot, domElem, collada);
				if (arr) {
					if (arr.length == 3) {
						matrix = sglMulM4(matrix, sglScalingM4V(arr));
					}
				}
			break;

			case "instance_geometry" :
				instanceGeometries = getColladaInstanceGeometry(domRoot, domElem, collada);
				if (instanceGeometries) {
					node.instanceGeometries = node.instanceGeometries.concat(instanceGeometries);
				}
			break;

			default :
			break;
		}

		domElem = domElem.nextSibling;
	}
	
	node.matrix = matrix;
	//ros_error(matrix);
	
	if ((node.children.length <= 0) && (node.instanceGeometries.length <= 0)) {
		return null;
	}

	return node;
}

function getColladaInstanceVisualScene(domRoot, domInstanceVisualScene, collada) {
	if (!domRoot || !domInstanceVisualScene || !collada) return null;

	var domNodes = [ ];
	var domElem = domInstanceVisualScene.firstChild;
	while (domElem) {
		if (domElem.tagName == "node") {
			domNodes.push(domElem);
		}
		domElem = domElem.nextSibling;
	}

	var domNodesCount = domNodes.length;
	if (domNodesCount <= 0) return null;

	var instanceVisualScene = new ColladaInstanceVisualScene();
	instanceVisualScene.id = domInstanceVisualScene.getAttribute("id");
	if (!instanceVisualScene.id) return null;

	for (var i=0; i<domNodesCount; ++i) {
		var domNode = domNodes[i];
		if (!domNode) continue;

		var node = getColladaNode(domRoot, domNode, collada);
		if (!node) continue;

		instanceVisualScene.nodes.push(node);
	}

	if (instanceVisualScene.nodes.length <= 0) {
		return null;
	}

	return instanceVisualScene;
}

function getColladaInstanceVisualScenes(domRoot, collada) {
	if (!domRoot || !collada) return null;

	for (var instanceVisualSceneURL in collada.instanceVisualScenes) {
		var instanceVisualScene = getColladaInstanceVisualScene(domRoot, getXmlElementById(domRoot, instanceVisualSceneURL), collada);
		if (instanceVisualScene) {
			collada.instanceVisualScenes[instanceVisualSceneURL] = instanceVisualScene;
		}
		else {
			delete collada.instanceVisualScenes[instanceVisualSceneURL];
		}
	}
}

function getColladaScene(domRoot, domScene, collada) {
	if (!domRoot || !domScene || !collada) return null;

	var domInstanceVisualScenes = domScene.getElementsByTagName("instance_visual_scene");
	if (!domInstanceVisualScenes) return null;
	var domInstanceVisualScenesCount = domInstanceVisualScenes.length;
	if (domInstanceVisualScenesCount <= 0) return null;

	var scene = new ColladaScene();

	for (var i=0; i<domInstanceVisualScenesCount; ++i) {
		var domInstanceVisualScene = domInstanceVisualScenes[i];
		if (!domInstanceVisualScene) continue;

		var instanceVisualSceneURL = domInstanceVisualScene.getAttribute("url");
		if (!instanceVisualSceneURL) continue;

		instanceVisualSceneURL = instanceVisualSceneURL.substr(1);
		if (instanceVisualSceneURL.length <= 0) continue;

		collada.instanceVisualScenes[instanceVisualSceneURL] = null;
		scene.instanceVisualSceneIDs.push(instanceVisualSceneURL);
	}

	if (scene.instanceVisualSceneIDs.length <= 0) {
		return null;
	}

	return scene;
}

function getColladaScenes(domRoot, collada) {
	if (!domRoot || !collada) return;

	var domScenes = domRoot.getElementsByTagName("scene");
	if (!domScenes) return;
	var domScenesCount = domScenes.length;
	if (domScenesCount <= 0) return;

	collada.scenes = { };
	var k = 0;

	for (var i=0; i<domScenesCount; ++i) {
		var domScene = domScenes[i];
		if (!domScene) continue;

		var scene = getColladaScene(domRoot, domScene, collada);
		if (!scene) continue;

		collada.scenes["scene_" + k] = scene;
		k++;
	}
}

function getColladaAsset(domRoot, domAsset, collada) {
  if (!domRoot || !domAsset || !collada) return null;
  
  var domElem  = domAsset.firstChild;

  while (domElem) {
    switch (domElem.tagName) {
      case "unit" :
        var unitName = domElem.getAttribute("name");
        if (!unitName) continue;
        var unitScale = domElem.getAttribute("meter");
        if (!unitScale) continue;
        collada.unitScale = unitScale;
      break;

      default :
      break;
    }

    domElem = domElem.nextSibling;
  }
}
  
function getColladaAssets(domRoot, collada) {
  if (!domRoot || !collada) return;

  var domAssets = domRoot.getElementsByTagName("asset");
  if (!domAssets) return;
  var domAssetsCount = domAssets.length;
  if (domAssetsCount <= 0) return;

  for (var i=0; i<domAssetsCount; ++i) {
    var domAsset = domAssets[i];
    getColladaAsset(domRoot, domAsset, collada);
  }
}

function getCollada(domRoot) {
	if (!domRoot) return null;

	var collada = new ColladaDocument();
	
	getColladaAssets                (domRoot, collada);
	getColladaScenes                (domRoot, collada);
	getColladaInstanceVisualScenes  (domRoot, collada);
	getColladaGeometries            (domRoot, collada);
	getColladaMaterials             (domRoot, collada);
	getColladaTextures              (domRoot, collada);

	return collada;
}
