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

/**
 * A Module for visualization.
 * @namespace
 */
ros.visualization = ros.visualization || {};

/**
 * unproject a point into 3D space
 */
ros.visualization.unproject = function(winx, winy, winz,
                               model, proj,
                               viewport) {
  var obj = new sglV3();
  
  /* matrice de transformation */
  var m, A;
  var input = new sglV4();
  var output = new sglV4();

  /* transformation coordonnees normalisees entre -1 et 1 */
  input[0] = (winx - viewport[0]) * 2.0 / viewport[2] - 1.0;
  input[1] = (winy - viewport[1]) * 2.0 / viewport[3] - 1.0;
  input[2] = 2.0 * winz - 1.0;
  input[3] = 1.0;

  /* calcul transformation inverse */
  A = sglMulM4(proj, model);
  m = sglInverseM4(A);

  /* d'ou les coordonnees objets */
  output = sglMulM4V4(m, input);
  if (output[3] == 0.0)
   return null;
  
  obj[0] = output[0] / output[3];
  obj[1] = output[1] / output[3];
  obj[2] = output[2] / output[3];
  return obj;
};

/**
 * Takes a 4-by-4 matrix and a vector with 3 entries,
 * interprets the vector as a point, transforms that point by the matrix, and
 * returns the result as a vector with 3 entries.
 * @param {!o3djs.math.Matrix4} m The matrix.
 * @param {!o3djs.math.Vector3} v The point.
 * @return {!o3djs.math.Vector3} The transformed point.
 */
ros.visualization.transformPoint = function(m, v) {
  var v0 = v[0];
  var v1 = v[1];
  var v2 = v[2];

  var d = v0 * m[ 3] + v1 * m[ 7] + v2 * m[11] + m[15];
  return [(v0 * m[ 0] + v1 * m[ 4] + v2 * m[ 8] + m[12]) / d,
          (v0 * m[ 1] + v1 * m[ 5] + v2 * m[ 9] + m[13]) / d,
          (v0 * m[ 2] + v1 * m[ 6] + v2 * m[10] + m[14]) / d];
};

ros.visualization.getRotationMat = function(m) {
  var r = sglDupM4(m);
  r[3] = 0;
  r[7] = 0;
  r[11] = 0;
  r[12] = 0;
  r[13] = 0;
  r[14] = 0;

  return r;
};

ros.visualization.getPoseMatrix = function(position,orientation)
{
  var matrix = sglIdentityM4();
  var translation = sglTranslationM4V([position.x, position.y, position.z]);
  var rotation = sglGetQuatRotationM4([orientation.x, orientation.y, orientation.z, orientation.w]);
  matrix = sglMulM4(matrix, translation);
  matrix = sglMulM4(matrix, rotation);

  return matrix;
};





// include all urdf files at once
ros.include('visualization/webgl-debug');
ros.include('visualization/scenenode/scenenode');
ros.include('visualization/shader/shader');
ros.include('visualization/collada/collada');
ros.include('visualization/model/model');
ros.include('visualization/marker/marker');
ros.include('visualization/interactor/interactor');
ros.include('visualization/interactivemarker/interactivemarker');
ros.include('visualization/camera/camera');
ros.include('visualization/markermanager');
ros.include('visualization/interactivemarkermanager');
ros.include('visualization/visualizationmanager');
ros.include('visualization/pickmanager');
ros.include('visualization/cameraoverlay');
ros.include('visualization/sceneviewer');
ros.include('visualization/rayintersectioninfo');
ros.include('visualization/boundingbox');
ros.include('visualization/rightcontext');
