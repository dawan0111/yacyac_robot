import { mat4, mat3, quat, vec3 } from "gl-matrix";

export function createSE3Matrix(translation: vec3, quaternion: quat): mat4 {
  const rotationMatrix: mat3 = mat3.create();
  mat3.fromQuat(rotationMatrix, quaternion);

  const se3Matrix: mat4 = mat4.fromValues(
    rotationMatrix[0],
    rotationMatrix[1],
    rotationMatrix[2],
    translation[0],
    rotationMatrix[3],
    rotationMatrix[4],
    rotationMatrix[5],
    translation[1],
    rotationMatrix[6],
    rotationMatrix[7],
    rotationMatrix[8],
    translation[2],
    0,
    0,
    0,
    1
  );

  return se3Matrix;
}
