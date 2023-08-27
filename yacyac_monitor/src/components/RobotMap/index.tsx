// @ts-nocheck

import {
  FlyControls,
  OrbitControls,
  PerspectiveCamera,
  useTexture,
} from "@react-three/drei";
import { Canvas, useFrame, useLoader } from "@react-three/fiber";
import React from "react";

import {
  Camera,
  Color,
  ImageLoader,
  Mesh,
  TextureLoader,
  Quaternion,
} from "three";
import { GLTFLoader } from "three/examples/jsm/loaders/GLTFLoader";

export interface RobotMapProps {
  imageSrc: string;
  robotPose: mat4;
  origins: number[];
  markers: number[][];
}

export function RobotMapThreeJS({
  imageSrc,
  robotPose,
  origins,
  markers,
}: RobotMapProps) {
  const camera = React.useRef(null);
  const orbitControl = React.useRef(null);

  const robotRef = React.useRef(null);
  const robotMesh = useLoader(GLTFLoader, "/robot.glb");
  const mapMarker = useLoader(GLTFLoader, "/scene.gltf");

  const whiteColor = new Color(0xffffff);
  const planeTexture = useLoader(TextureLoader, imageSrc);
  const imageSource = planeTexture.source.data;

  const imageCenterXY = [
    (imageSource.width * 0.05) / 2,
    (imageSource.height * 0.05) / 2,
  ];
  const deltaCenterXY = [
    imageCenterXY[0] + origins[0],
    imageCenterXY[1] + origins[1],
  ];

  return (
    <>
      <ambientLight intensity={1.4} />
      <directionalLight />
      <PerspectiveCamera
        ref={camera}
        position={[
          (robotPose[3] - deltaCenterXY[0]) * -1,
          (robotPose[7] - deltaCenterXY[1]) * -1,
          0,
        ]}
        args={[75, window.innerWidth / window.innerHeight, 1, 1100]}
      >
        <OrbitControls
          rotateSpeed={0.5}
          enableDamping={false}
          reverseOrbit={false}
          enablePan={false}
          enableRotate={true}
          maxAzimuthAngle={0.8}
          minAzimuthAngle={-0.8}
          maxPolarAngle={2.8}
          minPolarAngle={0.7}
          enableZoom={true}
          ref={orbitControl}
        />
        <mesh position={[0, 0, 0]} rotation={[0, 0, 0]}>
          <planeGeometry
            args={[imageSource.width * 0.05, imageSource.height * 0.05]}
          />
          <meshBasicMaterial map={planeTexture} color={whiteColor} />
        </mesh>

        <primitive
          ref={robotRef}
          object={robotMesh.scene.clone()}
          position={[
            robotPose[3] - deltaCenterXY[0],
            robotPose[7] - deltaCenterXY[1],
            0,
          ]}
          rotation={[Math.PI / 2, 0, 0]}
          scale={0.06}
        />

        {markers.map((marker, index) => (
          <primitive
            key={index}
            // ref={(el) => {
            //   if (marker[0] === goal[0] && marker[1] === goal[1]) {
            //     itemsRef.current = el;
            //   }
            // }}
            object={mapMarker.scene.clone()}
            position={[
              marker[0] - deltaCenterXY[0],
              marker[1] - deltaCenterXY[1],
              1,
            ]}
            rotation={[Math.PI / 2, 0, 0]}
            scale={0.3}
          />
        ))}
      </PerspectiveCamera>
    </>
  );
}

export default function RobotMap(props: RobotMapProps) {
  return (
    <>
      <Canvas>
        <RobotMapThreeJS {...props} />
      </Canvas>
    </>
  );
}
